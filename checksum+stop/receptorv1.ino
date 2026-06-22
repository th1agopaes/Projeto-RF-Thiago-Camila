/* ============================================================
  Placa: ESP32 Protoboard B (Receptor)
  Versão: Checksum + Stop-and-Wait ARQ
  Pinos:
    GPIO13 DATA módulo RX Dados
    GPIO27 DATA módulo TX ACK
    GPIO32 SDA LCD I2C 16x2
    GPIO33 SCL LCD I2C 16x2

  Estrutura do frame:
    FLAG(1) | TYPE(1) | SEQ(1) | LEN(1) | DATA(0–16) | FCS(1)

  Tipos de frame:
    0x01 = DATA
    0x02 = ACK
    0x03 = END

  3 modos:
   ESPERA: aguarda cabeçalho de sessão
   TEXTO: recebe quadros de texto e exibe no LCD
   IMAGEM: recebe linhas bitmap e reconstrói a imagem

  ACK:
   Ao receber DATA válido: envia ACK após 2s
   Se receber retransmissão (seq já processado): reenvia ACK
   Frames com FCS inválido: descartados SEM ACK, o TX retransmite
============================================================
*/

#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  PINOS
#define PINO_RX_DADOS 13
#define PINO_TX_ACK 27
#define PINO_SDA_LCD 32
#define PINO_SCL_LCD 33

// Macros
#define FLAG_INICIO 0x7E
#define TIPO_DATA 0x01
#define TIPO_ACK 0x02
#define TIPO_END 0x03
#define SEQ_CABECALHO 0xFF
#define TOTAL_LINHAS_IMG 8
#define DELAY_ANTES_ACK 500 // 0,5s

// Esrtutura do frame
struct Quadro {
  uint8_t flag;
  uint8_t tipo;
  uint8_t seq;
  uint8_t len;
  uint8_t dados[16];
  uint8_t fcs;
};

/*
 Checksum
*/
class Checksum {
  public:
    static uint8_t calcular(const Quadro& q) {
      uint16_t soma = 0;

      soma += q.tipo;
      soma += q.seq;
      soma += q.len;

      for (int i = 0; i < q.len; i++) soma += q.dados[i];

      return (uint8_t)(soma & 0xFF);
    }
};

// Display:
//  LCD 16x2 I2C endereço 0x27
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
      escrever(1, "RF 433MHz pronto");
    }

    void limpar() { lcd.clear(); }

    // Escreve em uma linha (0 ou 1) no máximo 16 caracteres
    void escrever(uint8_t linha, const String& texto) {
      lcd.setCursor(0, linha);
      lcd.print("                ");
      lcd.setCursor(0, linha);
      lcd.print(texto.substring(0, 16));
    }

    void mostrarTexto(uint8_t seq, const String& texto) {
      escrever(0, "SEQ:" + String(seq));
      escrever(1, texto);
    }

    void mostrarLinhaImagem(uint8_t numLinha, const String& bits) {
      escrever(0, "IMG LIN" + String(numLinha) + "/7:");
      escrever(1, bits);
    }

    void mostrarErroFCS(uint8_t seq) {
      escrever(0, "ERRO FCS SEQ:" + String(seq));
      escrever(1, "Descartado");
    }

    void mostrarTextoOK() {
      escrever(0, "Texto recebido");
      escrever(1, "OK");
    }

    void mostrarImagemOK() {
      escrever(0, "Imagem recebida");
      escrever(1, "Completa");
    }

    void mostrarPronto() {
      escrever(0, "Aguardando TX...");
      escrever(1, "RF 433MHz pronto");
    }
};

//  CanalRF:
//    Recebe frame DATA via GPIO13
//    Envia frame ACK via GPIO27
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_DADOS, PINO_TX_ACK, 0) {}

    bool iniciar() { return driver.init(); }

    // Tenta receber um frame
    bool receberQuadro(Quadro& q) {
      uint8_t buffer[21];
      uint8_t tamanho = sizeof(buffer);
      if (!driver.recv(buffer, &tamanho)) return false;
      if (tamanho < 5) return false;

      q.flag = buffer[0];
      q.tipo = buffer[1];
      q.seq  = buffer[2];
      q.len  = buffer[3];
      if (q.len > 16) return false;

      for (int i = 0; i < q.len; i++) q.dados[i] = buffer[4 + i];
      q.fcs = buffer[4 + q.len];
      return true;
    }

    // Aguarda 0,5s e envia o ACK com o seq informado
    void enviarACK(uint8_t seq) {
      delay(DELAY_ANTES_ACK);
      uint8_t buffer[4] = { FLAG_INICIO, TIPO_ACK, seq, 0x00 };
      driver.send(buffer, 4);
      driver.waitPacketSent();
    }
};

// Receptor:
//   3 modos
//    ESPERA: aguarda cabeçalho DATA[0xFF] com "TEXTO"/"IMAGEM"
//    TEXTO: recebe frame DATA e exibe no LCD
//    IMAGEM: recebe frame DATA e armazena no bufferImagem

//  Tratamento de ACK:
//    DATA novo e válido: processa + envia ACK(seq)
//    Retransmissão do TX: reenvia ACK(seq-1) sem reprocessar
//    FCS inválido: descarta e não envia ACK
//    Cabeçalho repetido: reenvia ACK(0xFF) sem reconfigurar
//    END no modo ESPERA: ignora (duplicata do 3x)
class Receptor {
  private:

    enum Modo { ESPERA, TEXTO, IMAGEM };

    CanalRF& canal;
    Display& display;

    Modo    modo;
    uint8_t seqEsperada;
    char    bufferImagem[TOTAL_LINHAS_IMG][9];
    uint8_t linhasRecebidas;

  public:
    Receptor(CanalRF& c, Display& d)
      : canal(c), display(d),
        modo(ESPERA), seqEsperada(0), linhasRecebidas(0)
    {}

    // Chamado continuamente no loop() — sem delay() aqui!
    void processar() {
      Quadro q;
      if (!canal.receberQuadro(q)) return;

      // Valida FLAG
      if (q.flag != FLAG_INICIO) {
        Serial.println("[DESCARTE] FLAG invalida: 0x" + String(q.flag, HEX));
        return;
      }

      // Valida FCS
      uint8_t fcsCalculado = Checksum::calcular(q);
      if (fcsCalculado != q.fcs) {
        Serial.println("=================================");
        Serial.print("[ERRO-FCS] SEQ:" + String(q.seq));
        Serial.print(" | Calc:0x"); Serial.print(fcsCalculado, HEX);
        Serial.print(" | Rcb:0x");  Serial.println(q.fcs, HEX);
        Serial.println("[DESCARTE] Sem ACK, o TX vai retransmitir");
        display.mostrarErroFCS(q.seq);
        return;
      }

      //Roteia por tipo
      if (q.tipo == TIPO_DATA) tratarDATA(q);
      else if (q.tipo == TIPO_END) tratarEND();
    }

  private:
    // Trata quadro DATA
    void tratarDATA(const Quadro& q) {

      // Cabeçalho de sessão SEQ=0xFF
      if (q.seq == SEQ_CABECALHO) {
        char tipo[17] = {};
        memcpy(tipo, q.dados, q.len);
        tipo[q.len] = '\0';

        bool modoJaCorreto = (modo == TEXTO  && String(tipo) == "TEXTO") ||
                             (modo == IMAGEM && String(tipo) == "IMAGEM");

        if (!modoJaCorreto) {
          // Primeira recepção — configura modo e reseta estado
          Serial.println("=================================");
          Serial.println("[CAB] Sessao: \"" + String(tipo) + "\"");

          if (String(tipo) == "TEXTO") {
            modo = TEXTO;
            seqEsperada = 0;
            Serial.println("[CAB] Modo: TEXTO");

          } else if (String(tipo) == "IMAGEM") {
            modo = IMAGEM;
            seqEsperada = 0;
            linhasRecebidas = 0;
            memset(bufferImagem, 0, sizeof(bufferImagem));
            Serial.println("[CAB] Modo: IMAGEM");
          }
        }
        // Sempre responde ACK, seja primeira vez ou retransmissão
        canal.enviarACK(SEQ_CABECALHO);
        Serial.println("[ACK] Cabecalho confirmado");
        return;
      }

      // Quadro DATA normal

      // Rejeita se não recebeu cabeçalho ainda
      if (modo == ESPERA) {
        Serial.println("[DESCARTE] Dado sem cabecalho de sessao");
        return;
      }

      // Se é retransmissão do TX (seq já processado), reenvia o ACK
      if (q.seq == (uint8_t)(seqEsperada - 1)) {
        canal.enviarACK(q.seq);
        Serial.println("[ACK-RE] Reenviado ACK SEQ:" + String(q.seq));
        return;
      }

      // Descarta se fora de ordem
      if (q.seq != seqEsperada) {
        Serial.print("[FORA-ORDEM] Esp:" + String(seqEsperada));
        Serial.println(" | Rcb:" + String(q.seq) + "  descartando");
        return;
      }

      // Quadro novo e válido

      // Extrai texto
      char texto[17] = {};
      memcpy(texto, q.dados, q.len);
      texto[q.len] = '\0';

      // Log
      Serial.println("=================================");
      Serial.print("[RX-OK] SEQ:" + String(q.seq));
      Serial.print(" | FCS:0x"); Serial.print(q.fcs, HEX);
      Serial.println(" | \"" + String(texto) + "\"");

      // Envia ACK
      canal.enviarACK(q.seq);
      Serial.println("[ACK] Enviado SEQ:" + String(q.seq));

      seqEsperada++;

      // Processa conforme modo atual
      if (modo == TEXTO) {
        display.mostrarTexto(q.seq, String(texto));

      } else if (modo == IMAGEM) {
        if (linhasRecebidas < TOTAL_LINHAS_IMG) {
          strcpy(bufferImagem[linhasRecebidas], texto);
          display.mostrarLinhaImagem(linhasRecebidas, String(texto));
          linhasRecebidas++;
        }
      }
    }

    //  Trata quadro END
    void tratarEND() {
      // Ignora se está em ESPERA
      if (modo == ESPERA) {
        Serial.println("[END] Ignorado: modo ESPERA.");
        return;
      }

      Serial.println("=================================");

      if (modo == TEXTO) {
        Serial.println("[END-TEXTO] Transmissao de texto concluida");
        display.mostrarTextoOK();

      } else if (modo == IMAGEM) {
        Serial.println("[END-IMAGEM] Transmissao de imagem concluida");
        imprimirImagem();
        display.mostrarImagemOK();
      }

      resetarEstado();
    }

    //  Imprime a imagem reconstruída no Serial
    void imprimirImagem() {
      Serial.println();
      Serial.println("=================================");
      Serial.println("   IMAGEM RECONSTRUIDA    ");
      Serial.println("=================================");
      Serial.println("|  Col:  01234567          |");
      for (int i = 0; i < TOTAL_LINHAS_IMG; i++) {
        String lin = "║  L" + String(i) + ":   ";
        lin += String(bufferImagem[i]);
        lin += "  ║";
        Serial.println(lin);
      }
      Serial.println("=================================");
    }

    //  Reseta estado para aguardar próxima sessão
    void resetarEstado() {
      modo = ESPERA;
      seqEsperada = 0;
      linhasRecebidas = 0;
      memset(bufferImagem, 0, sizeof(bufferImagem));

      delay(3000);
      display.mostrarPronto();
      Serial.println("[INFO] Aguardando proxima transmissao...\n");
    }
};

//  Instâncias globais
CanalRF  canal;
Display  display;
Receptor receptor(canal, display);

//  setup
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("========================================");
  Serial.println("  RECEPTOR: Stop-and-Wait ARQ");
  Serial.println("  FCS: Checksum (soma dos bytes mod 256)");
  Serial.println("  GPIO13=RX_DADOS e GPIO27=TX_ACK");
  Serial.println("  GPIO32=SDA_LCD  e GPIO33=SCL_LCD");
  Serial.println("========================================");

  display.iniciar();
  Serial.println("[OK] LCD inicializado");

  if (!canal.iniciar()) {
    Serial.println("[ERRO] Falha ao inicializar driver RF");
    display.escrever(0, "ERRO: RF falhou");
    while (true) delay(1000);
  }

  Serial.println("[OK] Driver RF inicializado");
  Serial.println("[INFO] Aguardando quadros do transmissor...\n");
}

// loop
void loop() {
  receptor.processar();
}

