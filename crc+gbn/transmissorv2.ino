// ================================================================
//  Placa: ESP32 Protoboard A (Transmissor)
//  Versão: CRC-8 + Go-Back-N ARQ
//
//  Pinos:
//    GPIO13  DATA módulo TX Dados
//    GPIO27  DATA módulo RX ACK
//
//  Estrutura do frame:
//    FLAG(1) | TIPO(1) | SEQ(1) | LEN(1) | DADOS(0-16) | FCS(1)
//
//  Tipos de frame:
//    0x01 = DATA
//    0x02 = ACK
//    0x03 = NAK
//    0x04 = END
//
//  Protocolo de sessao:
//    TX envia cabecalho DATA[SEQ=0xFF] com "TEXTO" ou "IMAGEM"
//    Receptor responde ACK(0xFF) e ajusta o modo. So entao comeca
//    a transmissao dos dados
//
//  Go-Back-N ARQ (janela = 4):
//    TX envia ate JANELA frames sem esperar ACK
//    Se receber NAK(seq) ou timeout, retransmite a partir do seq
//    com falha ate o fim da janela atual ("volta N")
//    Receptor aceita apenas frames em ordem, qualquer frame
//    fora de ordem gera NAK e descarte
//
//  CRC-8 (polinomio 0x07: x^8 + x^2 + x + 1):
//    Calculado sobre os campos: tipo + seq + len + dados[]
// ================================================================

#include <RH_ASK.h>
#include <SPI.h>

//  PINOS
#define PINO_TX_DADOS   13
#define PINO_RX_ACK     27

//  MACROS DO PROTOCOLO
#define FLAG_INICIO     0x7E
#define TIPO_DATA       0x01
#define TIPO_ACK        0x02
#define TIPO_NAK        0x03
#define TIPO_END        0x04
#define SEQ_CAB         0xFF    // Sequencia especial: cabecalho de sessao
#define TIMEOUT_MS      2500    // 2,5s sem ACK/NAK dispara Go-Back-N
#define GBN_JANELA      4       // Tamanho da janela do protocolo Go-Back-N
#define MAX_DADOS       16      // Tamanho maximo do campo de dados em bytes

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
//
//  Implementa CRC-8 com polinomio gerador 0x07 (x^8 + x^2 + x + 1)
//
//  Como funciona:
//    Comeca com crc = 0x00
//    Para cada byte dos dados:
//      XOR do byte com o registrador crc
//      Para cada bit (8 vezes):
//        Se o bit mais significativo for 1: crc = (crc << 1) XOR 0x07
//        Senao:          crc = crc << 1
//    O resultado é o FCS de 8 bits
//
//  Motivo  do CRC ser melhor que o Checksum:
//    Checksum: soma simples e falha quando dois bytes erram
//    se compensando mutuamente
//    CRC: divisao polinomial detecta todos os erros de
//    rajada de ate 8 bits (grau do polinomio)
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
//  Classe: CanalRF
//    Canal de Dados TX via GPIO13
//    Canal de ACK/NAK RX via GPIO27
// ================================================================
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_ACK, PINO_TX_DADOS, 0) {}

    bool iniciar() { return driver.init(); }

    // Serializa e transmite o frame pelo canal de dados
    void enviarFrame(const Frame& q) {
      uint8_t tamanho = 4 + q.len + 1;   // flag + tipo + seq + len + dados + fcs
      uint8_t buf[21];
      
      buf[0] = q.flag;
      buf[1] = q.tipo;
      buf[2] = q.seq;
      buf[3] = q.len;
      for (int i = 0; i < q.len; i++) buf[4 + i] = q.dados[i];
      buf[4 + q.len] = q.fcs;
      
      driver.send(buf, tamanho);
      driver.waitPacketSent();
    }

    // Escuta o canal de ACK por ate TIMEOUT_MS ms
    // Preenche tipo e seq do frame recebido
    // Retorna true se recebeu um frame valido
    bool ouvirResposta(uint8_t& tipoRcb, uint8_t& seqRcb,
                       unsigned long timeoutMs = TIMEOUT_MS) {
      unsigned long inicio = millis();
      while (millis() - inicio < timeoutMs) {
        uint8_t buf[10];
        uint8_t tam = sizeof(buf);
        if (driver.recv(buf, &tam)) {
          if (tam >= 3 && buf[0] == FLAG_INICIO) {
            tipoRcb = buf[1];
            seqRcb  = buf[2];
            return true;
          }
        }
      }
      return false;
    }
};

// ================================================================
//  Classe: GoBackN
//
//  Conceito:
//    O transmissor mantem uma janela deslizante de ate 4 posicoes
//    de frames enviados mas nao confirmados.
//    Quando recebe ACK(n), confirma todos os frames ate n (cumulativo)
//    Quando recebe NAK(n) ou timeout, retransmite a partir de n
//    ate o ultimo frame enviado ("Go-Back-N")
//
//  Diferenca do Stop-and-Wait:
//    Stop-and-Wait: janela=1 (envia 1, espera ACK, envia 1...)
//    Go-Back-N:     janela=4 (envia 4 de uma vez, recupera erros)
//
//  transmitirLote() aceita array de frames genericos com dados
//  em bytes brutos (uint8_t*), permitindo texto e imagem real
// ================================================================
class GoBackN {
  private:
    CanalRF& canal;

    uint8_t seqBase;              // Primeiro seq nao confirmado
    uint8_t seqProximo;           // Proximo seq a enviar
    Frame   janela[GBN_JANELA];   // Buffer dos frames enviados
    bool    ocupado[GBN_JANELA];  // Slot da janela em uso?
    bool    erroJaSimulado;       // Erro simulado ocorre apenas 1 vez

    int  slot(uint8_t seq)    { return seq % GBN_JANELA; }
    int  naoConfirmados()     { return (int)(seqProximo - seqBase); }
    bool cheia()              { return naoConfirmados() >= GBN_JANELA; }

  public:
    GoBackN(CanalRF& c) : canal(c), seqBase(0), seqProximo(0),
                          erroJaSimulado(false) {
      memset(ocupado, false, sizeof(ocupado));
    }

    void resetar() {
      seqBase        = 0;
      seqProximo     = 0;
      erroJaSimulado = false;
      memset(ocupado, false, sizeof(ocupado));
    }


    //  enviarCabecalho()
    //  Anuncia o tipo de sessao ("TEXTO" ou "IMAGEM").
    //  Retransmite indefinidamente ate receber ACK(0xFF).
    void enviarCabecalho(const char* tipo) {
      Frame q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = SEQ_CAB;
      q.len  = strlen(tipo);
      memcpy(q.dados, tipo, q.len);
      q.fcs  = CRC8::calcular(q);

      Serial.println(F("\n===================================="));
      Serial.println("[CAB] Anunciando sessao: \"" + String(tipo) + "\"");
      Serial.println("[CAB] CRC-8 do cabecalho: 0x" + String(q.fcs, HEX));

      int tent = 0;
      while (true) {
        tent++;
        canal.enviarFrame(q);
        uint8_t tipoR, seqR;
        if (canal.ouvirResposta(tipoR, seqR, TIMEOUT_MS)) {
          if (tipoR == TIPO_ACK && seqR == SEQ_CAB) {
            Serial.println("[CAB] Confirmado pelo receptor. (Tent:" +
                           String(tent) + ")");
            return;
          }
        }
        Serial.println("[CAB] Sem resposta, retransmitindo... (Tent:" +
                       String(tent) + ")");
      }
    }

    //  transmitirLote()
    //  Transmite N frames de dados usando Go-Back-N.
    //  Cada frame carrega ate MAX_DADOS bytes brutos.
    //
    //  dadosPorFrame[i] aponta para os bytes do frame i
    //  lenPorFrame[i]   e o tamanho em bytes do frame i
    //  total            e o numero total de frames
    //  simularErroNoSeq e o SEQ que sera corrompido (-1 = nenhum)
    void transmitirLote(const uint8_t* dadosPorFrame[], const uint8_t lenPorFrame[],
                        int total, int simularErroNoSeq = -1) {
      resetar();
      int confirmados = 0;

      while (confirmados < total) {

        // Fase 1: Enche a janela com frames novos
        while (!cheia() && seqProximo < (uint8_t)total) {
          bool simErro = ((int)seqProximo == simularErroNoSeq);
          prepararEEnviar(seqProximo,
                          dadosPorFrame[seqProximo],
                          lenPorFrame[seqProximo],
                          simErro);
          seqProximo++;
        }

        // Fase 2: Aguarda ACK ou NAK
        uint8_t tipoR, seqR;
        bool recebeu = canal.ouvirResposta(tipoR, seqR, TIMEOUT_MS);

        if (!recebeu) {
          // Timeout: retransmite toda a janela a partir de seqBase
          Serial.println(F("===================================="));
          Serial.println("[GBN-TIMEOUT] Sem resposta. Go-Back-N a partir de SEQ:" +
                         String(seqBase));
          retransmitirDe(seqBase, dadosPorFrame, lenPorFrame, total, -1);

        } else if (tipoR == TIPO_ACK) {
          // ACK cumulativo: confirma todos ate seqR inclusive
          if (seqR >= seqBase && seqR < seqProximo) {
            Serial.println("[ACK-CUMULATIVO] Confirmados SEQ:0 ate SEQ:" +
                           String(seqR) + " | Janela desliza para " +
                           String((int)seqR + 1));
            confirmados = (int)seqR + 1;
            seqBase     = seqR + 1;
          }
          // ACK duplicado (seqR < seqBase): ignora silenciosamente

        } else if (tipoR == TIPO_NAK) {
          // NAK: receptor pediu retransmissao a partir de seqR
          Serial.println(F("===================================="));
          Serial.println("[NAK] Receptor pediu retransmissao de SEQ:" +
                         String(seqR) + " em diante");
          retransmitirDe(seqR, dadosPorFrame, lenPorFrame, total, -1);
        }
      }
    }

    //  enviarFim()
    //  Envia frame END 3 vezes para garantir chegada
    void enviarFim(const char* descricao) {
      Frame q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_END;
      q.seq  = seqProximo;
      q.len  = 0;
      q.fcs  = CRC8::calcular(q);

      Serial.println(F("===================================="));
      Serial.println("[TX] Enviando END (" + String(descricao) + ") 3 vezes...");
      for (int i = 0; i < 3; i++) {
        canal.enviarFrame(q);
        delay(400);
      }
      Serial.println("[TX] END enviado com sucesso.");
    }

  private:

    // Monta, armazena na janela e envia um frame com dados brutos
    // simErro=true corrompe o primeiro byte dos dados (apenas 1 vez)
    void prepararEEnviar(uint8_t seq, const uint8_t* dados,
                         uint8_t len, bool simErro) {
      Frame& q = janela[slot(seq)];
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = seq;
      q.len  = len;
      memcpy(q.dados, dados, len);
      q.fcs  = CRC8::calcular(q);
      ocupado[slot(seq)] = true;

      bool aplicarErro = simErro && !erroJaSimulado;
      if (aplicarErro) {
        erroJaSimulado = true;
        q.dados[0] ^= 0xFF;   // Corrompe 1 byte — CRC vai divergir
        Serial.println(F("===================================="));
        Serial.println(">>> [ERRO-SIM] Corrompendo SEQ:" + String(seq) +
                       " — CRC vai detectar! (1 vez apenas)");
      }

      Serial.println(F("===================================="));
      Serial.print("[TX] SEQ:" + String(q.seq));
      Serial.print(" | LEN:" + String(q.len) + "B");
      Serial.print(" | CRC8:0x"); Serial.print(q.fcs, HEX);
      Serial.print(" | Janela:[" + String(seqBase) + "-" +
                   String((int)seqBase + GBN_JANELA - 1) + "]");
      if (aplicarErro) Serial.print(" [CORROMPIDO]");
      Serial.println();

      canal.enviarFrame(q);
      delay(100);
    }

    // Retransmite frames de 'de' ate seqProximo-1
    void retransmitirDe(uint8_t de, const uint8_t* dadosPorFrame[],
                        const uint8_t lenPorFrame[], int total,
                        int simErroNoSeq) {
      for (uint8_t s = de; s < seqProximo && s < (uint8_t)total; s++) {
        Serial.println("[GBN-RETX] Retransmitindo SEQ:" + String(s));
        prepararEEnviar(s, dadosPorFrame[s], lenPorFrame[s],
                        (int)s == simErroNoSeq);
      }
    }
};

// ================================================================
//  Classe: Menu
//  Interface serial para o usuario
// ================================================================
class Menu {
  public:
    void exibir() {
      Serial.println();
      Serial.println(F("===================================="));
      Serial.println(F("  TRANSMISSOR RF 433MHz — V4        "));
      Serial.println(F("  Algoritmos: CRC-8 + Go-Back-N ARQ "));
      Serial.println(F("===================================="));
      Serial.println(F("  1. Enviar mensagens de texto       "));
      Serial.println(F("  2. Enviar imagem real              "));
      Serial.println(F("  3. Simular erro (CRC detecta)      "));
      Serial.println(F("  4. Demo completa (texto + imagem)  "));
      Serial.println(F("===================================="));
      Serial.print(F("Escolha uma opcao: "));
    }

    char aguardarEscolha() {
      while (!Serial.available()) {}
      char op = Serial.read();
      while (Serial.available()) Serial.read();
      Serial.println(op);
      return op;
    }

    // Le uma mensagem de ate MAX_DADOS caracteres
    String lerUmaMsg(int numero) {
      Serial.print("  Mensagem " + String(numero) + " (max 16 chars): ");
      while (!Serial.available()) {}
      String t = Serial.readStringUntil('\n');
      t.trim();
      if (t.length() == 0) t = "vazio";
      if (t.length() > MAX_DADOS) t = t.substring(0, MAX_DADOS);
      Serial.println("  -> \"" + t + "\"");
      return t;
    }
};

// ================================================================
//
//  Representacao visual (1=pixel aceso, 0=apagado):
//
//  L0: 0 0 1 1 1 1 0 0   0 0 1 1 1 1 0 0
//  L1: 0 1 0 0 0 0 1 0   0 1 0 0 0 0 1 0
//  L2: 1 0 1 0 0 1 0 1   1 0 1 0 0 1 0 1
//  L3: 1 0 0 0 0 0 0 1   1 0 0 0 0 0 0 1
//  L4: 1 0 1 0 0 1 0 1   1 0 1 0 0 1 0 1
//  L5: 1 0 0 1 1 0 0 1   1 0 0 1 1 0 0 1
//  L6: 0 1 0 0 0 0 1 0   0 1 0 0 0 0 1 0
//  L7: 0 0 1 1 1 1 0 0   0 0 1 1 1 1 0 0
//
//  Transmitida em 2 frames de 8 bytes cada (16 bytes total)
//  Frame 0 = coluna 0-7 (lado esquerdo da imagem)
//  Frame 1 = coluna 8-15 (lado direito da imagem)
// ================================================================

// Frame 0: lado esquerdo (8 linhas, 8 bits por linha)
static const uint8_t imgFrame0[8] = {
  0b00111100,   // L0
  0b01000010,   // L1
  0b10100101,   // L2
  0b10000001,   // L3
  0b10100101,   // L4
  0b10011001,   // L5
  0b01000010,   // L6
  0b00111100    // L7
};

// Frame 1: lado direito (espelhado para compor imagem 16x8)
static const uint8_t imgFrame1[8] = {
  0b00111100,   // L0
  0b01000010,   // L1
  0b10100101,   // L2
  0b10000001,   // L3
  0b10100101,   // L4
  0b10011001,   // L5
  0b01000010,   // L6
  0b00111100    // L7
};

// Ponteiros e tamanhos para transmitirLote()
static const uint8_t* imgDados[2] = { imgFrame0, imgFrame1 };
static const uint8_t  imgLens[2]  = { 8, 8 };
static const int      IMG_TOTAL   = 2;   // 2 frames de imagem

// ================================================================
//  Mensagens da demo completa
// ================================================================
static const char* msgDemoStr[] = {
  "Ola",
  "RF 433MHz",
  "CRC-8 OK",
  "Go-Back-N",
  "Forouzan",
  "AAAAAAA"
};
static const int TOTAL_MSG_DEMO = siAzeof(msgDemoStr) / sizeof(msgDemoStr[0]);

// Converte array de strings para formato de transmitirLote()
static const uint8_t* msgDemoDados[6];
static uint8_t        msgDemoLens[6];

void prepararMsgDemo() {
  for (int i = 0; i < TOTAL_MSG_DEMO; i++) {
    msgDemoDados[i] = (const uint8_t*)msgDemoStr[i];
    msgDemoLens[i]  = strlen(msgDemoStr[i]);
  }
}

// ================================================================
//  OBJETOS GLOBAIS
// ================================================================
CanalRF canal;
GoBackN protocolo(canal);
Menu    menu;

// ================================================================
//  ACAO 1: Usuario digita 4 mensagens e envia como lote
// ================================================================
void acao_enviarTexto() {
  Serial.println(F("\n===================================="));
  Serial.println(F("  ENVIO DE TEXTO: CRC-8 + Go-Back-N "));
  Serial.println(F("===================================="));
  Serial.println(F("[INFO] Digite 4 mensagens para enviar."));
  Serial.println(F("[INFO] Go-Back-N envia todas na janela."));

  // Le 4 mensagens do usuario
  String msgs[4];
  for (int i = 0; i < 4; i++) {
    msgs[i] = menu.lerUmaMsg(i + 1);
  }

  // Converte para formato de transmitirLote()
  const uint8_t* dados[4];
  uint8_t        lens[4];
  for (int i = 0; i < 4; i++) {
    dados[i] = (const uint8_t*)msgs[i].c_str();
    lens[i]  = msgs[i].length();
  }

  Serial.println(F("\n[INFO] Iniciando transmissao Go-Back-N..."));
  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(dados, lens, 4);
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== TEXTO CONCLUIDO ==="));
}

// ================================================================
//  ACAO 2: Envia imagem real (2 frames de 8 bytes cada)
// ================================================================
void acao_enviarImagem() {
  Serial.println(F("\n===================================="));
  Serial.println(F("  ENVIO DE IMAGEM: CRC-8 + Go-Back-N"));
  Serial.println(F("===================================="));
  Serial.println(F("[INFO] Imagem: smiley 16x8 pixels"));
  Serial.println(F("[INFO] Transmitida em 2 frames de 8 bytes"));

  protocolo.enviarCabecalho("IMAGEM");
  protocolo.transmitirLote(imgDados, imgLens, IMG_TOTAL);
  protocolo.enviarFim("IMAGEM");
  Serial.println(F("=== IMAGEM CONCLUIDA ==="));
}

// ================================================================
//  ACAO 3: Simulacao de erro — CRC detecta, Go-Back-N recupera
// ================================================================
void acao_simularErro() {
  Serial.println(F("\n===================================="));
  Serial.println(F("  SIMULACAO DE ERRO CRC-8 + GBN     "));
  Serial.println(F("===================================="));
  Serial.println(F("[INFO] SEQ:1 sera corrompido 1 vez."));
  Serial.println(F("[INFO] Receptor calcula CRC, detecta divergencia,"));
  Serial.println(F("[INFO] envia NAK(1). Go-Back-N retransmite de SEQ:1."));

  static const char* simStr[] = {
    "Antes erro",
    "Corrompida",
    "Apos erro",
    "Recuperado!"
  };
  static const uint8_t* simDados[4];
  static uint8_t        simLens[4];
  for (int i = 0; i < 4; i++) {
    simDados[i] = (const uint8_t*)simStr[i];
    simLens[i]  = strlen(simStr[i]);
  }

  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(simDados, simLens, 4, 1);   // Corrompe SEQ:1
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== SIMULACAO CONCLUIDA — CRC+GBN funcionou! ==="));
}

// ================================================================
//  ACAO 4: Demo completa — texto com erro + imagem
// ================================================================
void acao_demoCompleta() {
  Serial.println(F("\n===================================="));
  Serial.println(F("  DEMO COMPLETA INICIADA            "));
  Serial.println(F("  Fase 1: Texto | Fase 2: Imagem    "));
  Serial.println(F("===================================="));

  // Fase 1: Texto (com erro no SEQ:2)
  Serial.println(F("\n=== FASE 1: TEXTO (SEQ:2 corrompido) ==="));
  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(msgDemoDados, msgDemoLens, TOTAL_MSG_DEMO, 2);
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== FIM FASE 1 ==="));

  delay(1500);

  // Fase 2: Imagem real
  Serial.println(F("\n=== FASE 2: IMAGEM REAL 16x8 ==="));
  protocolo.enviarCabecalho("IMAGEM");
  protocolo.transmitirLote(imgDados, imgLens, IMG_TOTAL);
  protocolo.enviarFim("IMAGEM");
  Serial.println(F("=== FIM FASE 2 ==="));

  Serial.println(F("\n===================================="));
  Serial.println(F("  DEMO COMPLETA ENCERRADA           "));
  Serial.println(F("===================================="));
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("===================================="));
  Serial.println(F("  TRANSMISSOR: CRC-8 + Go-Back-N   "));
  Serial.println(F("  GPIO13=TX_DADOS | GPIO27=RX_ACK  "));
  Serial.println("  Janela GBN = " + String(GBN_JANELA) + " frames");
  Serial.println(F("===================================="));

  prepararMsgDemo();

  if (!canal.iniciar()) {
    Serial.println(F("[ERRO] Falha ao inicializar driver RF"));
    while (true) delay(1000);
  }

  Serial.println(F("[OK] Driver RF 433MHz inicializado"));
  Serial.println(F("[INFO] Aguardando 3s para o receptor ficar pronto..."));
  delay(3000);
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  menu.exibir();
  char op = menu.aguardarEscolha();

  switch (op) {
    case '1': acao_enviarTexto();    break;
    case '2': acao_enviarImagem();   break;
    case '3': acao_simularErro();    break;
    case '4': acao_demoCompleta();   break;
    default:
      Serial.println(F("[AVISO] Opcao invalida. Digite 1, 2, 3 ou 4."));
      break;
  }
}
