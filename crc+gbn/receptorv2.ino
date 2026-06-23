// ================================================================
//  Placa: ESP32  Protoboard B (Receptor)
//  Versão: CRC-8 + Go-Back-N ARQ
//
//  Pinos:
//    GPIO13  DATA módulo RX Dados 
//    GPIO27 DATA módulo TX ACK 
//    GPIO32 SDA do LCD I2C 16x2
//    GPIO33  SCL do LCD I2C 16x2
//
//    3 modos:
//    ESPERA aguarda cabeçalho de sessão
//    TEXTO recebe frame de texto, exibe no LCD
//    IMAGEM recebe linhas bitmap, reconstrói a imagem
//
//   Go-Back-N no receptor:
//     Recebe apenas frame em ordem (seqEsperada)
//     Frame com CRC inválido envia NAK(seq esperado)
//     Frame fora de ordem envia NAK(seq esperado)
//     Frame válido e em ordem processa e envia ACK cumulativo
//     Retransmissão recebida reenvia ACK do último confirmado
//
//  CRC-8 (polinômio 0x07 — x^8 + x^2 + x + 1):
//    Idêntico ao do transmissor para os valores serem iguas
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

//  MACROS
#define FLAG_INICIO       0x7E
#define TIPO_DATA         0x01
#define TIPO_ACK          0x02
#define TIPO_NAK          0x03
#define TIPO_END          0x04
#define SEQ_CAB           0xFF
#define TOTAL_LINHAS_IMG  8
#define MAX_DADOS         16
#define DELAY_ACK_MS      300   // ms antes de enviar ACK

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
//  CLASSE: CRC8
//  Idêntica à do transmissor.
//  Polinômio 0x07 (x^8 + x^2 + x + 1)
// ================================================================
class CRC8 {
  public:
    static uint8_t calcular(const Frame& q) {
      uint8_t crc = 0x00;

      crc = _processar(crc, q.tipo);
      crc = _processar(crc, q.seq);
      crc = _processar(crc, q.len);
      for (int i = 0; i < q.len; i++) {
        crc = _processar(crc, q.dados[i]);
      }

      return crc;
    }

  private:
    static uint8_t _processar(uint8_t crc, uint8_t byte) {
      crc ^= byte;
      for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
          crc = (crc << 1) ^ 0x07;
        } else {
          crc <<= 1;
        }
      }
      return crc;
    }
};

// ================================================================
//  CLASSE: Display
//  LCD 16x2 I2C  endereço 0x27
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

    void mostrarSessão(const String& tipo) {
      escrever(0, "Sessao: " + tipo);
      escrever(1, "GBN pronto");
    }

    void mostrarTexto(uint8_t seq, const String& texto) {
      escrever(0, "SEQ:" + String(seq) + " recebido");
      escrever(1, texto);
    }

    void mostrarLinhaImagem(uint8_t linha, const String& bits) {
      escrever(0, "IMG L" + String(linha) + "/7 OK");
      escrever(1, bits);
    }

    void mostrarErroCRC(uint8_t seq) {
      escrever(0, "CRC FALHOU SEQ:" + String(seq));
      escrever(1, "NAK enviado");
    }

    void mostrarNAK(uint8_t seq) {
      escrever(0, "FORA ORDEM:" + String(seq));
      escrever(1, "NAK enviado");
    }

    void mostrarTextoOK(uint8_t total) {
      escrever(0, "TEXTO OK");
      escrever(1, String(total) + " frame receb.");
    }

    void mostrarImagemOK() {
      escrever(0, "IMAGEM OK");
      escrever(1, "8 linhas receb.");
    }

    void mostrarPronto() {
      escrever(0, "Aguardando TX...");
      escrever(1, "CRC8 + GBN");
    }
};

// ================================================================
//  CLASSE: CanalRF
//  Recebe frame DATA via GPIO13 (RX Dados)
//  Envia frame ACK/NAK via GPIO27 (TX ACK)
// ================================================================
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_DADOS, PINO_TX_ACK, 0) {}

    bool iniciar() { return driver.init(); }

    // Tenta receber um Frame
    // Retorna true se chegou um frame válido
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

    // Envia ACK cumulativo
    void enviarACK(uint8_t seq) {
      delay(DELAY_ACK_MS);
      uint8_t buf[4] = { FLAG_INICIO, TIPO_ACK, seq, 0x00 };
      driver.send(buf, 4);
      driver.waitPacketSent();
    }

    // Envia NAK
    void enviarNAK(uint8_t seq) {
      delay(DELAY_ACK_MS);
      uint8_t buf[4] = { FLAG_INICIO, TIPO_NAK, seq, 0x00 };
      driver.send(buf, 4);
      driver.waitPacketSent();
    }
};

// ================================================================
//  CLASSE: Receptor
//  3 modos:
//    ESPERA aguarda cabeçalho DATA[SEQ=0xFF]
//    TEXTO recebe frame DATA e exibe no LCD
//    IMAGEM recebe frame DATA e armazena no buffer
//
//  Lógica Go-Back-N no receptor
//    O receptor só aceita frame em ordem
//    Qualquer coisa fora de ordem manda NAK
//    CRC inválido NAK(seqEsperada) e o TX vai retransmitir
//    tudo a partir do Frame que falhou
//
//  ACK cumulativo:
//    ACK(n) confirma todos os frame de 0 até n inclusive
// ================================================================
class Receptor {
  private:
    enum Modo { ESPERA, TEXTO, IMAGEM };

    CanalRF& canal;
    Display& display;

    Modo    modo;
    uint8_t seqEsperada;
    uint8_t ultimoACK;
    bool    esperandoCabecalho;

    // Buffer de imagem 8 linhas de 8 bits + '\0'
    char    bufferImagem[TOTAL_LINHAS_IMG][9];
    uint8_t linhasRecebidas;

    // Estatísticas para exibir no final
    uint16_t totalRecebidos;
    uint16_t totalErroCRC;
    uint16_t totalForaOrdem;

  public:
    Receptor(CanalRF& c, Display& d)
      : canal(c), display(d),
        modo(ESPERA), seqEsperada(0), ultimoACK(255),
        esperandoCabecalho(true), linhasRecebidas(0),
        totalRecebidos(0), totalErroCRC(0), totalForaOrdem(0)
    {}

    void processar() {
      Frame q;
      if (!canal.receberFrame(q)) return;

      // Valida FLAG
      if (q.flag != FLAG_INICIO) {
        Serial.println("[DESCARTE] FLAG inválida: 0x" + String(q.flag, HEX));
        return;
      }

      //  Roteia por tipo 
      if      (q.tipo == TIPO_DATA) tratarDATA(q);
      else if (q.tipo == TIPO_END)  _tratarEND(q);
    }

  private:

    //  tratarDATA()
    void tratarDATA(const Frame& q) {

      //  Cabeçalho de sessão SEQ=0xFF 
      if (q.seq == SEQ_CAB) {
        tratarCABECALHO(q);
        return;
      }

      //  Rejeita dado sem cabeçalho 
      if (modo == ESPERA) {
        Serial.println("[DESCARTE] Dado sem cabeçalho de sessão");
        return;
      }

      //  Verifica CRC 
      uint8_t crcCalc = CRC8::calcular(q);
      if (crcCalc != q.fcs) {
        totalErroCRC++;
        Serial.println(F("========================================"));
        Serial.print("[ERRO-CRC] SEQ:" + String(q.seq));
        Serial.print(" | Calc:0x"); Serial.print(crcCalc, HEX);
        Serial.print(" | Rcb:0x");  Serial.println(q.fcs, HEX);
        Serial.println("[NAK] Enviando NAK(" + String(seqEsperada) +
                       "), TX vai retransmitir a partir de SEQ:" +
                       String(seqEsperada));
        display.mostrarErroCRC(q.seq);
        canal.enviarNAK(seqEsperada);
        return;
      }

      //  Verifica se é retransmissão já processada 
      // (TX retransmite toda a janela; pode chegar de novo o que já foi confirmado)
      if (q.seq < seqEsperada) {
        Serial.println("[ACK-RE] Retransmissao de SEQ:" + String(q.seq) +
                       ", reenviando ACK(" + String(ultimoACK) + ")");
        canal.enviarACK(ultimoACK);
        return;
      }

      //  Verifica se está fora de ordem 
      if (q.seq != seqEsperada) {
        totalForaOrdem++;
        Serial.println(F("========================================"));
        Serial.println("[FORA-ORDEM] Esp:" + String(seqEsperada) +
                       " | Rcb:" + String(q.seq) +
                       ", NAK(" + String(seqEsperada) + ")");
        display.mostrarNAK(q.seq);
        canal.enviarNAK(seqEsperada);
        return;
      }

      //  Frame válido, em ordem e com CRC correto 

      // Extrai texto
      char texto[MAX_DADOS + 1] = {};
      memcpy(texto, q.dados, q.len);
      texto[q.len] = '\0';

      totalRecebidos++;

      Serial.println(F("========================================"));
      Serial.print("[RX-OK] SEQ:" + String(q.seq));
      Serial.print(" | CRC8:0x"); Serial.print(q.fcs, HEX);
      Serial.println(" | \"" + String(texto) + "\"");

      // Processa conforme modo
      if (modo == TEXTO) {
        display.mostrarTexto(q.seq, String(texto));

      } else if (modo == IMAGEM) {
        if (linhasRecebidas < TOTAL_LINHAS_IMG) {
          strcpy(bufferImagem[linhasRecebidas], texto);
          display.mostrarLinhaImagem(linhasRecebidas, String(texto));
          linhasRecebidas++;
        }
      }

      // Envia ACK cumulativo
      ultimoACK   = q.seq;
      seqEsperada = q.seq + 1;
      canal.enviarACK(ultimoACK);
      Serial.println("[ACK] Enviado ACK(" + String(ultimoACK) +
                     ") confirmados SEQ:0 ate SEQ:" + String(ultimoACK));
    }

    // 
    //  tratarCABECALHO()
    // 
    void tratarCABECALHO(const Frame& q) {
      // Verifica CRC do cabeçalho também
      uint8_t crcCalc = CRC8::calcular(q);
      if (crcCalc != q.fcs) {
        Serial.println("[ERRO-CRC-CAB] Cabecalho corrompido, ignorando");
        return;
      }

      char tipo[17] = {};
      memcpy(tipo, q.dados, q.len);
      tipo[q.len] = '\0';

      bool modoCorreto = (modo == TEXTO  && String(tipo) == "TEXTO") ||
                         (modo == IMAGEM && String(tipo) == "IMAGEM");

      if (!modoCorreto) {
        // Primeira recepção — configura modo
        Serial.println(F("========================================"));
        Serial.println("[CAB] Sessão iniciada: \"" + String(tipo) + "\"");

        if (String(tipo) == "TEXTO") {
          modo        = TEXTO;
          seqEsperada = 0;
          ultimoACK   = 255;
          Serial.println(F("[CAB] Modo: TEXTO | Go-Back-N ativo"));

        } else if (String(tipo) == "IMAGEM") {
          modo            = IMAGEM;
          seqEsperada     = 0;
          ultimoACK       = 255;
          linhasRecebidas = 0;
          memset(bufferImagem, 0, sizeof(bufferImagem));
          Serial.println(F("[CAB] Modo: IMAGEM | Go-Back-N ativo"));
        }

        display.mostrarSessão(String(tipo));
      }

      // Sempre responde ACK ao cabeçalho (primeira vez ou retransmissão)
      canal.enviarACK(SEQ_CAB);
      Serial.println("[ACK] Cabeçalho confirmado. CRC-8:0x" +
                     String(q.fcs, HEX));
    }

    // 
    //  _tratarEND()
    // 
    void _tratarEND(const Frame& q) {
      if (modo == ESPERA) {
        Serial.println(F("[END] Ignorado, modo ESPERA"));
        return;
      }

      Serial.println(F("========================================"));

      if (modo == TEXTO) {
        Serial.println(F("[END-TEXTO] Transmissao de texto concluida"));
        imprimirEstatisticas();
        display.mostrarTextoOK(totalRecebidos);

      } else if (modo == IMAGEM) {
        Serial.println(F("[END-IMAGEM] Transmissao de imagem concluida"));
        _imprimirImagem();
        imprimirEstatisticas();
        display.mostrarImagemOK();
      }

      resetarEstado();
    }

    // 
    //  _imprimirImagem()
    // 
    void _imprimirImagem() {
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("║     IMAGEM RECONSTRUIDA      ║"));
      Serial.println(F("========================================╣"));
      Serial.println(F("║  Col:  0 1 2 3 4 5 6 7       ║"));
      Serial.println(F("========================================"));
      for (int i = 0; i < TOTAL_LINHAS_IMG; i++) {
        // Formata com espaços entre os bits para ficar mais legível
        String lin = "║  L" + String(i) + ":   ";
        for (int c = 0; c < 8; c++) {
          lin += bufferImagem[i][c];
          if (c < 7) lin += " ";
        }
        lin += "    ║";
        Serial.println(lin);
      }
      Serial.println(F("========================================"));
    }

    //  imprimirEstatisticas()
    void imprimirEstatisticas() {
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("║    ESTATISTICAS DA Sessão    ║"));
      Serial.println(F("========================================"));
      Serial.println("║  frame OK       : " +
                     _pad(String(totalRecebidos), 3) + "           ║");
      Serial.println("║  Erros de CRC     : " +
                     _pad(String(totalErroCRC), 3) + "           ║");
      Serial.println("║  Fora de ordem    : " +
                     _pad(String(totalForaOrdem), 3) + "           ║");
      Serial.println(F("========================================"));
    }

    String _pad(String s, int n) {
      while ((int)s.length() < n) s = " " + s;
      return s;
    }

    //  resetarEstado()
    void resetarEstado() {
      modo            = ESPERA;
      seqEsperada     = 0;
      ultimoACK       = 255;
      linhasRecebidas = 0;
      totalRecebidos  = 0;
      totalErroCRC    = 0;
      totalForaOrdem  = 0;
      memset(bufferImagem, 0, sizeof(bufferImagem));

      delay(3000);
      display.mostrarPronto();
      Serial.println(F("[INFO] Aguardando próxima transmissão...\n"));
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

  Serial.println(F("=========================================="));
  Serial.println(F("  RECEPTOR: CRC-8 + Go-Back-N ARQ   "));
  Serial.println(F("  GPIO13 = RX_DADOS e GPIO27 = TX_ACK        "));
  Serial.println(F("  GPIO32 = SDA_LCD  e GPIO33 = SCL_LCD       "));
  Serial.println(F("=========================================="));

  display.iniciar();
  Serial.println(F("[OK] LCD I2C inicializado"));

  if (!canal.iniciar()) {
    Serial.println(F("[ERRO] Falha ao inicializar driver RF"));
    display.escrever(0, "ERRO: RF falhou");
    display.escrever(1, "Reinicie");
    while (true) delay(1000);
  }

  Serial.println(F("[OK] Driver RF 433MHz inicializado"));
  Serial.println(F("[INFO] Aguardando frame do transmissor...\n"));
}

// ================================================================
//  LOOP — sem delay() aqui para não perder mensagens!
// ================================================================
void loop() {
  receptor.processar();
}
