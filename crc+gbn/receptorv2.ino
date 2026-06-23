// ================================================================
//  Placa: ESP32 Protoboard B (Receptor)
//  Versão: CRC-8 + Go-Back-N ARQ
//
//  Pinos:
//    GPIO13  DATA módulo RX Dados
//    GPIO27  DATA módulo TX ACK
//    GPIO32  SDA do LCD I2C 16x2
//    GPIO33  SCL do LCD I2C 16x2
//
//  3 modos de operação:
//    ESPERA  aguarda cabeçalho de sessão DATA[SEQ=0xFF]
//    TEXTO   recebe frames de texto e exibe no LCD
//    IMAGEM  recebe frames de bytes e reconstrói a imagem
//
//  Go-Back-N no receptor:
//    Aceita apenas frames em ordem (seqEsperada)
//    Frame com CRC inválido  → envia NAK(seqEsperada)
//    Frame fora de ordem     → envia NAK(seqEsperada)
//    Frame válido e em ordem → processa + ACK cumulativo
//    Retransmissão já vista  → reenvia ACK(ultimoACK)
//
//  CRC-8 (polinômio 0x07 — x^8 + x^2 + x + 1):
//    Idêntico ao do transmissor para os valores baterem
// ================================================================

#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  PINOS
#define PINO_RX_DADOS   13
#define PINO_TX_ACK     27
#define PINO_SDA_LCD    32
#define PINO_SCL_LCD    33

//  MACROS DO PROTOCOLO
#define FLAG_INICIO       0x7E
#define TIPO_DATA         0x01
#define TIPO_ACK          0x02
#define TIPO_NAK          0x03
#define TIPO_END          0x04
#define SEQ_CAB           0xFF
#define MAX_DADOS         16
#define DELAY_ACK_MS      300    // ms antes de enviar ACK/NAK

//  Tamanho máximo do buffer de imagem (frames × bytes por frame)
//  2 frames × 8 bytes = 16 bytes (imagem 16x8 pixels)
#define MAX_FRAMES_IMG    2
#define BYTES_POR_FRAME   8
#define MAX_BYTES_IMG     (MAX_FRAMES_IMG * BYTES_POR_FRAME)

//  ESTRUTURA DO FRAME
struct Frame {
  uint8_t flag;
  uint8_t tipo;
  uint8_t seq;
  uint8_t len;
  uint8_t dados[MAX_DADOS];
  uint8_t fcs;
};

// ================================================================
//  Classe: CRC8
//  Idêntica à do transmissor — polinômio 0x07 (x^8 + x^2 + x + 1)
//  Calculado sobre: tipo + seq + len + dados[]
// ================================================================
class CRC8 {
  public:
    static uint8_t calcular(const Frame& q) {
      uint8_t crc = 0x00;
      crc = processar(crc, q.tipo);
      crc = processar(crc, q.seq);
      crc = processar(crc, q.len);
      for (int i = 0; i < q.len; i++) {
        crc = processar(crc, q.dados[i]);
      }
      return crc;
    }

  private:
    static uint8_t processar(uint8_t crc, uint8_t byte) {
      crc ^= byte;
      for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0x07;
        else            crc <<= 1;
      }
      return crc;
    }
};

// ================================================================
//  Classe: Display
//  LCD 16x2 I2C — endereço 0x27
//  SDA: GPIO32 | SCL: GPIO33 | VCC: 3.3V
// ================================================================
class Display {
  private:
    LiquidCrystal_I2C lcd;

  public:
    Display() : lcd(0x27, 16, 2) {}

    void iniciar() {
      Wire.begin(PINO_SDA_LCD, PINO_SCL_LCD);
      lcd.init();
      lcd.backlight();
      limpar();
      escrever(0, "Aguardando TX...");
      escrever(1, "CRC8 + GBN");
    }

    void limpar() { lcd.clear(); }

    void escrever(uint8_t linha, const String& texto) {
      lcd.setCursor(0, linha);
      lcd.print("                ");
      lcd.setCursor(0, linha);
      lcd.print(texto.substring(0, 16));
    }

    void mostrarSessao(const String& tipo) {
      escrever(0, "Sessao: " + tipo);
      escrever(1, "GBN pronto");
    }

    void mostrarTexto(uint8_t seq, const String& texto) {
      escrever(0, "SEQ:" + String(seq) + " recebido");
      escrever(1, texto);
    }

    void mostrarFrameImagem(uint8_t frame, uint8_t total) {
      escrever(0, "IMG frame " + String(frame) + "/" + String(total - 1));
      escrever(1, String(BYTES_POR_FRAME) + " bytes OK");
    }

    void mostrarErroCRC(uint8_t seq) {
      escrever(0, "CRC FALHOU:" + String(seq));
      escrever(1, "NAK enviado");
    }

    void mostrarNAK(uint8_t seq) {
      escrever(0, "FORA ORDEM:" + String(seq));
      escrever(1, "NAK enviado");
    }

    void mostrarTextoOK(uint16_t total) {
      escrever(0, "TEXTO OK!");
      escrever(1, String(total) + " frames receb.");
    }

    void mostrarImagemOK(int bytes) {
      escrever(0, "IMAGEM OK!");
      escrever(1, String(bytes) + " bytes receb.");
    }

    void mostrarPronto() {
      escrever(0, "Aguardando TX...");
      escrever(1, "CRC8 + GBN");
    }
};

// ================================================================
//  Classe: CanalRF
//  Recebe frames DATA via GPIO13 (RX Dados)
//  Envia frames ACK/NAK via GPIO27 (TX ACK)
// ================================================================
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_DADOS, PINO_TX_ACK, 0) {}

    bool iniciar() { return driver.init(); }

    // Tenta receber um frame — retorna true se chegou algo válido
    bool receberFrame(Frame& q) {
      uint8_t buf[21];
      uint8_t tam = sizeof(buf);
      if (!driver.recv(buf, &tam)) return false;
      if (tam < 5) return false;

      q.flag = buf[0];
      q.tipo = buf[1];
      q.seq  = buf[2];
      q.len  = buf[3];
      if (q.len > MAX_DADOS) return false;

      for (int i = 0; i < q.len; i++) q.dados[i] = buf[4 + i];
      q.fcs = buf[4 + q.len];
      return true;
    }

    // Envia ACK cumulativo — confirma todos os frames até seq
    void enviarACK(uint8_t seq) {
      delay(DELAY_ACK_MS);
      uint8_t buf[4] = { FLAG_INICIO, TIPO_ACK, seq, 0x00 };
      driver.send(buf, 4);
      driver.waitPacketSent();
    }

    // Envia NAK — solicita retransmissão a partir de seq
    void enviarNAK(uint8_t seq) {
      delay(DELAY_ACK_MS);
      uint8_t buf[4] = { FLAG_INICIO, TIPO_NAK, seq, 0x00 };
      driver.send(buf, 4);
      driver.waitPacketSent();
    }
};

// ================================================================
//  Classe: Receptor
//
//  Máquina de estados com 3 modos:
//    ESPERA  → aguarda cabeçalho DATA[SEQ=0xFF]
//    TEXTO   → recebe frames DATA e exibe texto no LCD
//    IMAGEM  → recebe frames DATA com bytes brutos da imagem
//
//  Lógica Go-Back-N no receptor:
//    Só aceita o próximo SEQ esperado (ordem estrita)
//    CRC inválido    → NAK(seqEsperada) sem processar
//    Fora de ordem   → NAK(seqEsperada) sem processar
//    Retransmissão   → reenvia ACK(ultimoACK) sem reprocessar
//    Frame correto   → processa + ACK(seq) cumulativo
//
//  ACK cumulativo:
//    ACK(n) confirma implicitamente todos os frames 0..n
// ================================================================
class Receptor {
  private:
    enum Modo { ESPERA, TEXTO, IMAGEM };

    CanalRF& canal;
    Display& display;

    Modo    modo;
    uint8_t seqEsperada;
    uint8_t ultimoACK;

    // Buffer da imagem recebida em bytes brutos
    uint8_t bufferImagem[MAX_BYTES_IMG];
    int     bytesRecebidos;
    int     framesImgRecebidos;

    // Estatísticas da sessão atual
    uint16_t totalFramesOK;
    uint16_t totalErroCRC;
    uint16_t totalForaOrdem;

  public:
    Receptor(CanalRF& c, Display& d)
      : canal(c), display(d),
        modo(ESPERA), seqEsperada(0), ultimoACK(255),
        bytesRecebidos(0), framesImgRecebidos(0),
        totalFramesOK(0), totalErroCRC(0), totalForaOrdem(0)
    {}

    // Chamado no loop() — processa 1 frame por ciclo se houver
    void processar() {
      Frame q;
      if (!canal.receberFrame(q)) return;

      // Valida FLAG
      if (q.flag != FLAG_INICIO) {
        Serial.println("[DESCARTE] FLAG invalida: 0x" + String(q.flag, HEX));
        return;
      }

      // Roteia por tipo
      if      (q.tipo == TIPO_DATA) tratarDATA(q);
      else if (q.tipo == TIPO_END)  tratarEND(q);
    }

  private:

    //  tratarDATA()
    //  Ponto de entrada para qualquer frame DATA recebido.
    //  Separa o cabeçalho de sessão dos frames de dados.
    void tratarDATA(const Frame& q) {

      // Cabeçalho de sessão SEQ=0xFF
      if (q.seq == SEQ_CAB) {
        tratarCABECALHO(q);
        return;
      }

      // Rejeita dado sem cabeçalho
      if (modo == ESPERA) {
        Serial.println("[DESCARTE] Dado sem cabecalho de sessao.");
        return;
      }

      // Verifica CRC
      uint8_t crcCalc = CRC8::calcular(q);
      if (crcCalc != q.fcs) {
        totalErroCRC++;
        Serial.println(F("========================================"));
        Serial.print("[ERRO-CRC] SEQ:" + String(q.seq));
        Serial.print(" | Calc:0x"); Serial.print(crcCalc, HEX);
        Serial.print(" | Rcb:0x");  Serial.println(q.fcs, HEX);
        Serial.println("[NAK] Enviando NAK(" + String(seqEsperada) +
                       ") — TX retransmite a partir de SEQ:" +
                       String(seqEsperada));
        display.mostrarErroCRC(q.seq);
        canal.enviarNAK(seqEsperada);
        return;
      }

      // Retransmissão já processada (TX reenviou janela)
      if (q.seq < seqEsperada) {
        Serial.println("[ACK-RE] Retransmissao de SEQ:" + String(q.seq) +
                       " ja processado — reenviando ACK(" +
                       String(ultimoACK) + ")");
        canal.enviarACK(ultimoACK);
        return;
      }

      // Fora de ordem
      if (q.seq != seqEsperada) {
        totalForaOrdem++;
        Serial.println(F("========================================"));
        Serial.println("[FORA-ORDEM] Esp:" + String(seqEsperada) +
                       " | Rcb:" + String(q.seq) +
                       " — NAK(" + String(seqEsperada) + ")");
        display.mostrarNAK(q.seq);
        canal.enviarNAK(seqEsperada);
        return;
      }

      // Frame válido, em ordem e CRC correto
      totalFramesOK++;

      Serial.println(F("========================================"));
      Serial.print("[RX-OK] SEQ:" + String(q.seq));
      Serial.print(" | CRC8:0x"); Serial.print(q.fcs, HEX);
      Serial.print(" | LEN:" + String(q.len) + "B");

      if (modo == TEXTO) {
        // Extrai e exibe o texto recebido
        char texto[MAX_DADOS + 1] = {};
        memcpy(texto, q.dados, q.len);
        texto[q.len] = '\0';
        Serial.println(" | \"" + String(texto) + "\"");
        display.mostrarTexto(q.seq, String(texto));

      } else if (modo == IMAGEM) {
        // Armazena os bytes brutos da imagem no buffer
        Serial.println(" | [bytes de imagem]");
        if (bytesRecebidos + q.len <= MAX_BYTES_IMG) {
          memcpy(bufferImagem + bytesRecebidos, q.dados, q.len);
          bytesRecebidos += q.len;
          framesImgRecebidos++;
          display.mostrarFrameImagem(framesImgRecebidos - 1, MAX_FRAMES_IMG);
        }
      }

      // Envia ACK cumulativo
      ultimoACK   = q.seq;
      seqEsperada = q.seq + 1;
      canal.enviarACK(ultimoACK);
      Serial.println("[ACK] Enviado ACK(" + String(ultimoACK) +
                     ") — confirmados SEQ:0 ate SEQ:" + String(ultimoACK));
    }

    //  tratarCABECALHO()
    //  Processa frame de anúncio de sessão DATA[SEQ=0xFF].
    //  Verifica CRC, configura o modo e responde ACK(0xFF).
    //  Se o modo já estiver correto (retransmissão), apenas
    //  reenvia o ACK sem reconfigurar o estado.
    void tratarCABECALHO(const Frame& q) {
      // Verifica CRC do próprio cabeçalho
      uint8_t crcCalc = CRC8::calcular(q);
      if (crcCalc != q.fcs) {
        Serial.println("[ERRO-CRC-CAB] Cabecalho corrompido — ignorando.");
        return;
      }

      char tipo[17] = {};
      memcpy(tipo, q.dados, q.len);
      tipo[q.len] = '\0';

      bool modoCorreto = (modo == TEXTO  && String(tipo) == "TEXTO") ||
                         (modo == IMAGEM && String(tipo) == "IMAGEM");

      if (!modoCorreto) {
        // Primeira recepção — configura o modo
        Serial.println(F("========================================"));
        Serial.println("[CAB] Sessao iniciada: \"" + String(tipo) + "\"");

        if (String(tipo) == "TEXTO") {
          modo        = TEXTO;
          seqEsperada = 0;
          ultimoACK   = 255;
          totalFramesOK = totalErroCRC = totalForaOrdem = 0;
          Serial.println(F("[CAB] Modo: TEXTO | Go-Back-N ativo"));

        } else if (String(tipo) == "IMAGEM") {
          modo               = IMAGEM;
          seqEsperada        = 0;
          ultimoACK          = 255;
          bytesRecebidos     = 0;
          framesImgRecebidos = 0;
          totalFramesOK = totalErroCRC = totalForaOrdem = 0;
          memset(bufferImagem, 0, sizeof(bufferImagem));
          Serial.println(F("[CAB] Modo: IMAGEM | Go-Back-N ativo"));
        }

        display.mostrarSessao(String(tipo));
      }

      // Sempre responde ACK ao cabeçalho
      canal.enviarACK(SEQ_CAB);
      Serial.println("[ACK] Cabecalho confirmado. CRC-8:0x" +
                     String(q.fcs, HEX));
    }

    //  tratarEND()
    //  Finaliza a sessão atual, exibe resultado e estatísticas.
    //  Reseta o estado para aguardar nova sessão.
    void tratarEND(const Frame& q) {
      if (modo == ESPERA) {
        Serial.println(F("[END] Ignorado — modo ESPERA."));
        return;
      }

      Serial.println(F("========================================"));

      if (modo == TEXTO) {
        Serial.println(F("[END-TEXTO] Transmissao de texto concluida."));
        imprimirEstatisticas();
        display.mostrarTextoOK(totalFramesOK);

      } else if (modo == IMAGEM) {
        Serial.println(F("[END-IMAGEM] Transmissao de imagem concluida."));
        imprimirImagem();
        imprimirEstatisticas();
        display.mostrarImagemOK(bytesRecebidos);
      }

      resetarEstado();
    }

    //  imprimirImagem()
    //  Exibe os bytes recebidos no Serial Monitor como:
    //    - Representação binária (linha visual da imagem)
    //    - Valor hexadecimal (para verificação)
    void imprimirImagem() {
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("       IMAGEM RECONSTRUIDA              "));
      Serial.println(F("========================================"));
      Serial.println("  Total: " + String(bytesRecebidos) +
                     " bytes | " + String(framesImgRecebidos) + " frames");
      Serial.println(F("  Col:  7 6 5 4 3 2 1 0   HEX"));
      Serial.println(F("----------------------------------------"));

      for (int i = 0; i < bytesRecebidos; i++) {
        String bin = "";
        for (int b = 7; b >= 0; b--) {
          bin += (bufferImagem[i] & (1 << b)) ? "1" : "0";
          if (b > 0) bin += " ";
        }
        String hex = "0x";
        if (bufferImagem[i] < 0x10) hex += "0";
        hex += String(bufferImagem[i], HEX);

        Serial.println("  L" + String(i) + ":  " + bin + "  " + hex);
      }
      Serial.println(F("========================================"));
    }

    //  imprimirEstatisticas()
    //  Exibe contadores da sessão: frames OK, erros CRC e
    //  frames fora de ordem — útil para análise do canal RF.
    void imprimirEstatisticas() {
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("       ESTATISTICAS DA SESSAO           "));
      Serial.println(F("========================================"));
      Serial.println("  Frames OK       : " + pad(String(totalFramesOK), 4));
      Serial.println("  Erros de CRC    : " + pad(String(totalErroCRC),  4));
      Serial.println("  Fora de ordem   : " + pad(String(totalForaOrdem),4));
      Serial.println(F("========================================"));
    }

    String pad(String s, int n) {
      while ((int)s.length() < n) s = " " + s;
      return s;
    }

    //  resetarEstado()
    //  Volta ao modo ESPERA e limpa todos os buffers e contadores
    //  para receber a próxima sessão do transmissor.
    void resetarEstado() {
      modo               = ESPERA;
      seqEsperada        = 0;
      ultimoACK          = 255;
      bytesRecebidos     = 0;
      framesImgRecebidos = 0;
      totalFramesOK      = 0;
      totalErroCRC       = 0;
      totalForaOrdem     = 0;
      memset(bufferImagem, 0, sizeof(bufferImagem));

      delay(3000);
      display.mostrarPronto();
      Serial.println(F("[INFO] Aguardando proxima transmissao...\n"));
    }
};

// ================================================================
//  OBJETOS GLOBAIS
// ================================================================
CanalRF  canal;
Display  display;
Receptor receptor(canal, display);

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("========================================"));
  Serial.println(F("  RECEPTOR: CRC-8 + Go-Back-N ARQ      "));
  Serial.println(F("  GPIO13=RX_DADOS | GPIO27=TX_ACK       "));
  Serial.println(F("  GPIO32=SDA_LCD  | GPIO33=SCL_LCD      "));
  Serial.println(F("========================================"));

  display.iniciar();
  Serial.println(F("[OK] LCD I2C inicializado."));

  if (!canal.iniciar()) {
    Serial.println(F("[ERRO] Falha ao inicializar driver RF."));
    display.escrever(0, "ERRO: RF falhou");
    display.escrever(1, "Reinicie");
    while (true) delay(1000);
  }

  Serial.println(F("[OK] Driver RF 433MHz inicializado."));
  Serial.println(F("[INFO] Aguardando frames do transmissor...\n"));
}

// ================================================================
//  LOOP — sem delay() aqui para nao perder frames!
// ================================================================
void loop() {
  receptor.processar();
}
