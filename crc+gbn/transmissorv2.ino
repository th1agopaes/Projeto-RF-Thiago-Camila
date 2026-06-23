// ================================================================
//  Placa: ESP32 Protoboard A (Transmissor)
//  Versão: CRC-8 + Go-Back-N ARQ
//
//  Pinos:
//    GPIO13  DATA módulo TX Dados
//    GPIO27  DATA módulo RX ACK
//
//  Estrutura do frame:
//    FLAG(1) | TIPO(1) | SEQ(1) | LEN(1) | DADOS(0–16) | FCS(1)
//
//  Tipos de frame:
//    0x01 = DATA 
//    0x02 = ACK
//    0x03 = NAK
//    0x04 = END
//
//  Protocolo de sessão:
//    TX envia cabeçalho DATA[SEQ=0xFF] com TEXTO ou IMAGEM
//    Receptor responde ACK(0xFF) e ajusta o modo. Só então começa
//    a transmissão dos dados
//
//  Go-Back-N ARQ (janela = 4):
//    TX envia até JANELA frames sem esperar ACK
//    Se receber NAK(seq) ou timeout, retransmite a partir do seq
//    com falha até o fim da janela atual ("volta N")
//    Receptor aceita apenas frames em ordem, qualquer frame
//    fora de ordem gera NAK e descarte
//
//  CRC-8 (polinômio 0x07: x^8 + x^2 + x + 1):
//    Calculado sobre os campos: tipo + seq + len + dados[]
// ================================================================

#include <RH_ASK.h>
#include <SPI.h>

//  PINOS
#define PINO_TX_DADOS   13
#define PINO_RX_ACK     27

//  MACROS
#define FLAG_INICIO     0x7E
#define TIPO_DATA       0x01
#define TIPO_ACK        0x02
#define TIPO_NAK        0x03
#define TIPO_END        0x04
#define SEQ_CAB         0xFF    // Sequência especial: cabeçalho de sessão
#define TIMEOUT_MS      2500    // 2,5s sem ACK dispara Go-Back-N
#define GBN_JANELA      4       // Tamanho da janela do protocolo Go-Back-N
#define MAX_DADOS       16      // Tamanho máximo do campo de dados

//  ESTRUTURA DO FRAME
struct frame {
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
//  Implementa CRC-8 com polinômio gerador 0x07 (x^8 + x^2 + x + 1)
//
//  Como funciona:
//     Começa com crc = 0x00
//     Para cada byte dos dados:
//        XOR do byte com o registrador do crc
//        Para cada bit (8 vezes):
//          Se o bit mais significativo for 1:
//            crc = (crc << 1) XOR 0x07
//          Senão:
//            crc = crc << 1
//     O resultado é o FCS de 8 bits
//
//    Checksum detecta erros simples mas falha quando dois bytes
//    erram se compensando. CRC detecta todos os erros de rajada 
//    até N bits, onde N é o grau do polinômio, no nosso caso 8
// ================================================================
class CRC8 {
  public:
    static uint8_t calcular(const frame& q) {
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
//  Classe: CanalRF
//    Canal de Dados TX via GPIO13
//    Canal de ACK RX via GPIO27
// ================================================================
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_ACK, PINO_TX_DADOS, 0) {}

    bool iniciar() { return driver.init(); }

    // Serializa e transmite o frame pelo canal de dados
    void enviarframe(const frame& q) {

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

    // Escuta o canal de ACK por até 2,5s.
    // Preenche os campos tipo e seq do frame que chegou
    // Retorna true se recebeu um frame válido
    bool ouvirResposta(uint8_t& tipoRecebido, uint8_t& seqRecebido,
                       unsigned long timeoutMs = TIMEOUT_MS) {
      unsigned long inicio = millis();
      while (millis() - inicio < timeoutMs) {
        uint8_t buf[10];
        uint8_t tam = sizeof(buf);
        if (driver.recv(buf, &tam)) {
          if (tam >= 3 && buf[0] == FLAG_INICIO) {
            tipoRecebido = buf[1];
            seqRecebido  = buf[2];
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
//    O transmissor mantém uma janela deslizante de até 4 posições
//    frames enviados mas não confirmados
//    Quando recebe ACK(n), confirma todos os frames até n
//    Quando recebe NAK(n) ou expira o timeout, vai N atrás
//    retransmite a partir do frame n até o último enviado
//
//  Diferença do Stop-and-Wait:
//    Stop-and-Wait: janela = 1 (envia 1, espera ACK, envia 1...)
//    Go-Back-N: janela = N (envia N de uma vez, recupera erros)
// ================================================================
class GoBackN {
  private:
    CanalRF& canal;

    uint8_t  seqBase;        // Primeiro seq não confirmado
    uint8_t  seqProximo;     // Próximo seq a enviar
    frame   janela[GBN_JANELA];   // Buffer de frames
    bool     ocupado[GBN_JANELA];  // Marca se o slot da janela está em uso
    bool     erroJaSimulado; // Garante que o erro simulado ocorre só 1 vez

    // Índice do slot na janela para um dado seq
    int slot(uint8_t seq) { return seq % GBN_JANELA; }

    // Quantos frames foram enviados mas nao confirmados
    int naoConfirmados() { return (int)(seqProximo - seqBase); }

    // Janela está cheia?
    bool cheia() { return naoConfirmados() >= GBN_JANELA; }

  public:
    GoBackN(CanalRF& c) : canal(c), seqBase(0), seqProximo(0), erroJaSimulado(false) {
      memset(ocupado, false, sizeof(ocupado));
    }

    void resetar() {
      seqBase         = 0;
      seqProximo      = 0;
      erroJaSimulado  = false;
      memset(ocupado, false, sizeof(ocupado));
    }

    //  enviarCabecalho()
    //  Envia o frame especial SEQ=0xFF anunciando o tipo
    //  de sessão (TEXTO ou IMAGEM). Fica retransmitindo
    //  até receber ACK(0xFF) do receptor.
    // ────────────────────────────────────────────────────────
    void enviarCabecalho(const char* tipo) {
      frame q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = SEQ_CAB;
      q.len  = strlen(tipo);
      memcpy(q.dados, tipo, q.len);
      q.fcs  = CRC8::calcular(q);

      Serial.println(F("\n===================================="));
      Serial.println("[CAB] Anunciando tipo da sessão: \"" + String(tipo) + "\"");
      Serial.println("[CAB] CRC-8 do cabeçalho: 0x" + String(q.fcs, HEX));

      int tent = 0;
      while (true) {
        tent++;
        canal.enviarframe(q);
        uint8_t tipoR, seqR;
        if (canal.ouvirResposta(tipoR, seqR, TIMEOUT_MS)) {
          if (tipoR == TIPO_ACK && seqR == SEQ_CAB) {
            Serial.println("[CAB] Confirmado pelo receptor (Tentativa:" + String(tent) + ")");
            return;
          }
        }
        Serial.println("[CAB] Sem resposta, retransmitindo... (Tentativa:" + String(tent) + ")");
      }
    }

    //  transmitirLote()
    //  Transmite um array de strings usando Go-Back-N
    //  totalframes = quantas strings há no array msgs[]
    //  simularErroNoSeq = seq que será corrompido (-1 = nenhum)
    void transmitirLote(const char* msgs[], int totalframes,
                        int simularErroNoSeq = -1) {
      resetar();
      int confirmados = 0;   // Quantos frames foram confirmados

      while (confirmados < totalframes) {
        // Fase 1: Encher a janela
        // Envia quantos frames couberem na janela (4)
        while (!cheia() && seqProximo < (uint8_t)totalframes) {
          prepararEEnviar(seqProximo, msgs[seqProximo],
                           (int)seqProximo == simularErroNoSeq);
          seqProximo++;
        }

        // Fase 2: Coletar respostas
        // Escuta por 2,5s por ACKs/NAKs
        uint8_t tipoR, seqR;
        bool recebeu = canal.ouvirResposta(tipoR, seqR, TIMEOUT_MS);

        if (!recebeu) {
          // Timeout: Go-Back-N a partir de seqBase
          Serial.println(F("===================================="));
          Serial.println("[GBN-TIMEOUT] Sem resposta. Retransmitindo a partir de SEQ:" +
                         String(seqBase));
          retransmitirDe(seqBase, msgs, totalframes, -1);  // Sem erro na retransmissao

        } else if (tipoR == TIPO_ACK) {
          // ACK cumulativo
          // ACK(n) confirma todos os frames até n inclusive
          if (seqR >= seqBase && seqR < seqProximo) {
            Serial.println("[ACK-CUMULATIVO] Confirmados SEQ:0 ate SEQ:" + String(seqR));
            confirmados   = (int)seqR + 1;
            seqBase       = seqR + 1;
          }
          // Se seqR < seqBase, é ACK duplicado, ignora

        } else if (tipoR == TIPO_NAK) {
          // NAK: receptor pediu retransmissão de seqR
          Serial.println(F("===================================="));
          Serial.println("[NAK] Receptor pediu retransmissao a partir de SEQ:" +
                         String(seqR));
          retransmitirDe(seqR, msgs, totalframes, -1);  // Sem erro na retransmissao
        }
      }
    }

    //  enviarFim()
    //  Envia frame END 3 vezes
    void enviarFim(const char* descricao) {
      frame q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_END;
      q.seq  = seqProximo;
      q.len  = 0;
      q.fcs  = CRC8::calcular(q);

      Serial.println(F("===================================="));
      Serial.println("[TX] Enviando END (" + String(descricao) + ") 3 vezes...");
      for (int i = 0; i < 3; i++) {
        canal.enviarframe(q);
        delay(400);
      }
      Serial.println("[TX] END enviado com sucesso");
    }

  private:

    // Monta e envia um frame, armazena na janela para eventual
    // retransmissão. simErro=true corrompe o FCS propositalmente
    void prepararEEnviar(uint8_t seq, const char* texto, bool simErro) {
      frame& q = janela[slot(seq)];
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = seq;
      q.len  = strlen(texto);
      memcpy(q.dados, texto, q.len);
      q.fcs  = CRC8::calcular(q);
      ocupado[slot(seq)] = true;

      uint8_t fcsExibido = q.fcs;
      bool aplicarErro = simErro && !erroJaSimulado;
      if (aplicarErro) {
        erroJaSimulado = true;
        q.dados[0] ^= 0xFF;   // Corrompe 1 byte do campo dados
        Serial.println(F("===================================="));
        Serial.println(">>> [ERRO-SIM] Corrompendo SEQ:" + String(seq) + " (1 vez apenas!)");
        fcsExibido = q.fcs;
      }

      Serial.println(F("===================================="));
      Serial.print("[TX] SEQ:" + String(q.seq));
      Serial.print(" | \"" + String(texto) + "\"");
      Serial.print(" | CRC8:0x"); Serial.print(fcsExibido, HEX);
      Serial.print(" | Janela:[" + String(seqBase) + "-" + String((int)seqBase + GBN_JANELA - 1) + "]");
      if (aplicarErro) Serial.print(" [CORROMPIDO]");
      Serial.println();

      canal.enviarframe(q);
      delay(100);
    }

    // Retransmite todos os frames a partir de 'De' até seqProximo-1
    void retransmitirDe(uint8_t de, const char* msgs[],
                         int total, int simErroNoSeq) {
      for (uint8_t s = de; s < seqProximo && s < (uint8_t)total; s++) {
        Serial.println("[GBN-RETX] Retransmitindo SEQ:" + String(s));
        prepararEEnviar(s, msgs[s],
                         (int)s == simErroNoSeq);
      }
    }
};

// ================================================================
//  Classe: Menu
//  Interface serial
// ================================================================
class Menu {
  public:
    void exibir() {
      Serial.println();
      Serial.println(F("===================================="));
      Serial.println(F("║    TRANSMISSOR RF 433MHz — V4  MENU      ║"));
      Serial.println(F("║    Algoritmos: CRC-8 + Go-Back-N ARQ     ║"));
      Serial.println(F("===================================="));
      Serial.println(F("║  1. Enviar mensagem de texto             ║"));
      Serial.println(F("║  2. Enviar imagem bitmap 8x8             ║"));
      Serial.println(F("║  3. Simular erro (CRC detecta)           ║"));
      Serial.println(F("║  4. Demo completa (texto + imagem)       ║"));
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

    String lerTexto() {
      Serial.print(F("Digite a mensagem (máximo de 16 caracteres): "));
      while (!Serial.available()) {}
      String t = Serial.readStringUntil('\n');
      t.trim();
      if (t.length() == 0) t = "vazio";
      if (t.length() > MAX_DADOS) t = t.substring(0, MAX_DADOS);
      Serial.println("-> \"" + t + "\"");
      return t;
    }
};

// ================================================================
//  DADOS PADRÃO
// ================================================================

// Mensagens usadas na demonstração completa de texto
const char* msgDemo[] = {
  "Ola",
  "RF 433MHz",
  "CRC-8 OK",
  "Go-Back-N",
  "Forouzan",
  "AAAAAA"
};
const int TOTAL_MSG_DEMO = sizeof(msgDemo) / sizeof(msgDemo[0]);

const char* imagemBitmap[] = {
  "00111100",
  "01000010",
  "10100101",
  "10000001",
  "10100101",
  "10011001",
  "01000010",
  "00111100"
};
const int TOTAL_LINHAS_IMG = sizeof(imagemBitmap) / sizeof(imagemBitmap[0]);

// ================================================================
//  OBJETOS GLOBAIS
// ================================================================
CanalRF  canal;
GoBackN  protocolo(canal);
Menu     menu;

//  AÇÃO 1: Texto digitado pelo usuário
void acao_enviarTexto() {
  String texto = menu.lerTexto();

  // Monta array com 1 elemento para reusar transmitirLote()
  const char* arr[1];
  String copia = texto;
  arr[0] = copia.c_str();

  Serial.println(F("\n==========================================="));
  Serial.println(F("         ENVIANDO TEXTO: CRC-8 + Go-Back-N    "));
  Serial.println(F("==========================================="));
  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(arr, 1);
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== TEXTO CONCLUIDO ==="));
}

//  AÇÃO 2: Imagem bitmap 8x8
void acao_enviarImagem() {
  Serial.println(F("\n==========================================="));
  Serial.println(F("   ENVIANDO IMAGEM BITMAP 8x8: CRC-8 + Go-Back-N "));
  Serial.println(F("==========================================="));
  protocolo.enviarCabecalho("IMAGEM");
  protocolo.transmitirLote(imagemBitmap, TOTAL_LINHAS_IMG);
  protocolo.enviarFim("IMAGEM");
  Serial.println(F("=== IMAGEM CONCLUIDA ==="));
}

//  AÇÃO 3: Simular erro proposital CRC deve detectar e GBN
//          deve retransmitir automaticamente
void acao_simularErro() {
  Serial.println(F("\n==========================================="));
  Serial.println(F("         SIMULACAO DE ERRO CRC-8           "));
  Serial.println(F("==========================================="));
  Serial.println(F("[INFO] SEQ:1 sera corrompido propositalmente."));
  Serial.println(F("[INFO] Receptor calcula CRC, detecta divergencia,"));
  Serial.println(F("[INFO] envia NAK(1). Go-Back-N retransmite."));

  const char* msgs[] = {
    "Antes do erro",
    "Erro aqui!",
    "Apos o erro",
    "Recuperado!"
  };

  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(msgs, 4, 1);   // Corrompe o SEQ:1
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== SIMULACAO CONCLUIDA — CRC+GBN funcionou! ==="));
}

//  AÇÃO 4: Demonstração completa — texto com erro + imagem
void acao_demoCompleta() {
  Serial.println(F("\n=========================================="));
  Serial.println(F("         DEMONSTRAÇÃO COMPLETA INICIADA           "));
  Serial.println(F("   Fase 1: Texto  |  Fase 2: Imagem       "));
  Serial.println(F("=========================================="));

  //  Fase 1: Texto (com erro proposital no SEQ:2)
  Serial.println(F("\n=== FASE 1: TEXTO (SEQ:2 será corrompido) ==="));
  protocolo.enviarCabecalho("TEXTO");
  protocolo.transmitirLote(msgDemo, TOTAL_MSG_DEMO, 2);
  protocolo.enviarFim("TEXTO");
  Serial.println(F("=== FIM FASE 1 ==="));

  delay(1500);

  // Fase 2: Imagem
  Serial.println(F("\n=== FASE 2: IMAGEM BITMAP 8x8 ==="));
  protocolo.enviarCabecalho("IMAGEM");
  protocolo.transmitirLote(imagemBitmap, TOTAL_LINHAS_IMG);
  protocolo.enviarFim("IMAGEM");
  Serial.println(F("=== FIM FASE 2 ==="));

  Serial.println(F("\n=========================================="));
  Serial.println(F("         DEMONSTRAÇÃO COMPLETA ENCERRADA          "));
  Serial.println(F("=========================================="));
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F("  TRANSMISSOR: CRC-8 + Go-Back-N ARQ "));
  Serial.println(F("  GPIO13=TX_DADOS e GPIO27=RX_ACK         "));
  Serial.println("  Janela GBN = " + String(GBN_JANELA) + " frames");
  Serial.println(F("=========================================="));

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
      Serial.println(F("[AVISO] Opcao invalida"));
      break;
  }
}
