
/* ============================================================
  Placa: ESP32 Protoboard A (Transmissor)
  Versão: Checksum + Stop-and-Wait ARQ
  Pinos:
    GPIO13 DATA módulo TX Dados
    GPIO27 DATA módulo RX ACK
    
  Estrutura do frame:
    FLAG(1) | TYPE(1) | SEQ(1) | LEN(1) | DATA(0–16) | FCS(1)

  Tipos de frame:
    0x01 = DATA
    0x02 = ACK 
    0x03 = END

 Protocolo de sessão:
    Antes dos dados, TX envia DATA[SEQ=0xFF] com "TEXTO" ou
   "IMAGEM". Receptor responde ACK(0xFF) e ajusta seu modo
   Só então a transmissão dos dados começa

  Stop-and-Wait ARQ:
   TX envia 1 frame e fica esperando a confirmação desse frame do RX (ACK);
   Não avança para o próximo frame até receber a confirmação;
   Se não receber o ACK dentro do tempo de 2 segundos, retransmite indefinidamente a cada 2s

  Checksum no TX:
    Antes do envio, soma todos os bytes da mensagem;
    O somatório é enviado pro campo FCS do frame;
    Erros podem passar  despercebidos devido a comutatividade da soma
 ============================================================
*/

#include <RH_ASK.h>
#include <SPI.h>

// Pinos
#define PINO_TX_DADOS 13
#define PINO_RX_ACK  27

// Macros
#define FLAG_INICIO 0x7E
#define TIPO_DATA 0x01
#define TIPO_ACK 0x02
#define TIPO_END 0x03
#define SEQ_CABECALHO 0xFF   // SEQ especial: quadro de anúncio de sessão
#define TIMEOUT_MS 2000

// Estrutura do frame
struct Quadro {
  uint8_t flag;
  uint8_t tipo;
  uint8_t seq;
  uint8_t len;
  uint8_t dados[16];
  uint8_t fcs;
};

/*
 Checksum:
  Calcula o FCS somando tipo + seq + len + dados (mod 256)
*/
class Checksum {
  public:
    static uint8_t calcularFCS(const Quadro& q) {
      uint16_t soma = 0;

      soma += q.tipo;
      soma += q.seq;
      soma += q.len;

      for (int i = 0; i < q.len; i++) soma += q.dados[i];

      return (uint8_t)(soma & 0xFF);
    }
};

// CanalRF:
//  TX Dados via GPIO13 e RX ACK via GPIO27
class CanalRF {
  private:
    RH_ASK driver;

  public:
    // RH_ASK(velocidade, pinoRX, pinoTX, pinoPTT)
    CanalRF() : driver(2000, PINO_RX_ACK, PINO_TX_DADOS, 0) {}

    bool iniciar() { return driver.init(); }

    // Serializa o frame e transmite pelo módulo TX Dados
    void enviarQuadro(const Quadro& q) {
      uint8_t tamanho = 4 + q.len + 1;
      uint8_t buffer[21];
      buffer[0] = q.flag;
      buffer[1] = q.tipo;
      buffer[2] = q.seq;
      buffer[3] = q.len;
      for (int i = 0; i < q.len; i++) buffer[4 + i] = q.dados[i];
      buffer[4 + q.len] = q.fcs;
      driver.send(buffer, tamanho);
      driver.waitPacketSent();
    }

    // Escuta o módulo RX ACK por até 2s
    // Retorna true se chegou um ACK válido com o campo seq correto
    bool aguardarACK(uint8_t seqEsperado) {
      unsigned long inicio = millis();
      while (millis() - inicio < TIMEOUT_MS) {
        uint8_t buffer[10];
        uint8_t tamanho = sizeof(buffer);
        if (driver.recv(buffer, &tamanho)) {
          if (tamanho >= 3 &&
              buffer[0] == FLAG_INICIO &&
              buffer[1] == TIPO_ACK &&
              buffer[2] == seqEsperado) {
            return true;
          }
        }
      }
      return false;
    }
};


/* Implementação do protocolo Stop-and-Wait ARQ:
    Envia 1 frame;
    Aguarda o ACK por 2s;
    Se não receber o ACK, retransmite o mesmo frame;
    Só avança para o próximo frame após o ACK ser confirmado
*/
class StopAndWait {
  private:
    CanalRF& canal;
    uint8_t sequencia;

  public:
    StopAndWait(CanalRF& c) : canal(c), sequencia(0) {}

    // Fica em loop retransmitindo até o receptor enviar o ACK
    void enviarCabecalho(const char* tipo) {
      Quadro q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = SEQ_CABECALHO;
      q.len  = strlen(tipo);
      memcpy(q.dados, tipo, q.len);
      q.fcs = Checksum::calcular(q);

      int tentativa = 0;
      Serial.println("[CAB] Anunciando tipo: \"" + String(tipo) + "\"");
      while (true) {
        tentativa++;
        canal.enviarQuadro(q);
        delay(50);
        if (canal.aguardarACK(SEQ_CABECALHO)) {
          Serial.println("[CAB] Confirmado pelo receptor. (Tent:" + String(tentativa) + ")");
          return;
        }
        Serial.println("[CAB] Aguardando confirmacao... (Tent:" + String(tentativa) + ")");
      }
    }

    // Envia um quadro frame e aguarda o ACK
    // simularErro=true, corrompe 1 byte para demonstração de detecção de erros
    // Só retorna quando o ACK for recebido
    void enviar(const char* texto, bool simularErro = false) {
      Quadro q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_DATA;
      q.seq  = sequencia;
      q.len  = strlen(texto);
      memcpy(q.dados, texto, q.len);
      q.fcs  = Checksum::calcular(q);

      if (simularErro) {
        q.dados[0] ^= 0xFF;
        Serial.println(">>> [ERRO-SIM] Corrompendo SEQ:" + String(sequencia) + " propositalmente!");
      }

      int tentativa = 0;
      while (true) {
        tentativa++;
        Serial.println("------------------------------------");
        Serial.print("[TX] SEQ:" + String(q.seq));
        Serial.print(" | \"" + String(texto) + "\"");
        Serial.print(" | FCS:0x"); Serial.print(q.fcs, HEX);
        if (simularErro) Serial.print(" [CORROMPIDO]");
        Serial.println(" | Tent:" + String(tentativa));

        canal.enviarQuadro(q);
        delay(50);

        if (canal.aguardarACK(sequencia)) {
          Serial.println("[ACK] Confirmado SEQ:" + String(sequencia));
          sequencia++;
          return;
        }
        Serial.println("[TIMEOUT] Sem ACK. Retransmitindo...");
      }
    }

    // Envia quadro END — enviado 3x para garantir chegada
    // (END não precisa de ACK, usamos redundância simples)
    void enviarFim(const char* descricao) {
      Quadro q;
      q.flag = FLAG_INICIO;
      q.tipo = TIPO_END;
      q.seq  = sequencia;
      q.len  = 0;
      q.fcs  = Checksum::calcular(q);

      Serial.println("------------------------------------");
      Serial.println("[TX] Enviando END (" + String(descricao) + ") 3x...");
      for (int i = 0; i < 3; i++) {
        canal.enviarQuadro(q);
        delay(400);
      }
      Serial.println("[TX] END enviado.");
    }

    void resetarSequencia() { sequencia = 0; }
};

//  Menu:
//   Interface serial para interagir
class Menu {
  public:
    void exibir() {
      Serial.println();
      Serial.println("===================================");
      Serial.println("|      TRANSMISSOR RF 433MHz MENU    |");
      Serial.println("=================================");
      Serial.println("|  1. Enviar mensagem de texto         |");
      Serial.println("|  2. Enviar imagem bitmap 8x8         |");
      Serial.println("|  3. Simular erro de transmissao      |");
      Serial.println("|  4. Executar demonstração completa           |");
      Serial.println("==================================");
      Serial.print("Escolha uma opcao: ");
    }

    char aguardarEscolha() {
      while (!Serial.available()) {}
      char opcao = Serial.read();
      while (Serial.available()) Serial.read();
      Serial.println(opcao);
      return opcao;
    }

    String lerTexto() {
      Serial.print("Digite a mensagem (max 16 chars): ");
      while (!Serial.available()) {}
      String texto = Serial.readStringUntil('\n');
      texto.trim();
      if (texto.length() == 0) texto = "vazio";
      if (texto.length() > 16) texto = texto.substring(0, 16);
      Serial.println("-> \"" + texto + "\"");
      return texto;
    }
};

//  DADOS padrão
const char* mensagensPadrao[] = {
  "Ola ESP32!",
  "RF 433MHz",
  "Checksum OK",
  "Stop & Wait",
  "Forouzan",
  "TCD 2026"
};
const int TOTAL_MSGS = sizeof(mensagensPadrao) / sizeof(mensagensPadrao[0]);

// Imagem bitmap 8x8
// Cada string = 1 linha = 1 quadro DATA
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
const int TOTAL_LINHAS = sizeof(imagemBitmap) / sizeof(imagemBitmap[0]);

//  Instâncias globais
CanalRF canal;
StopAndWait protocolo(canal);
Menu menu;

// Ação 1: Texto digitado
void acao_enviarTexto() {
  String texto = menu.lerTexto();
  Serial.println("\n=== ENVIANDO TEXTO ===");
  protocolo.resetarSequencia();
  protocolo.enviarCabecalho("TEXTO");
  protocolo.enviar(texto.c_str());
  protocolo.enviarFim("TEXTO");
  Serial.println("=== TEXTO CONCLUIDO ===");
}

//  Ação 2: Imagem bitmap 8x8
void acao_enviarImagem() {
  Serial.println("\n=== ENVIANDO IMAGEM BITMAP 8x8 ===");
  protocolo.resetarSequencia();
  protocolo.enviarCabecalho("IMAGEM");
  for (int i = 0; i < TOTAL_LINHAS; i++) {
    protocolo.enviar(imagemBitmap[i]);
    delay(100);
  }
  protocolo.enviarFim("IMAGEM");
  Serial.println("=== IMAGEM CONCLUIDA ===");
}

//  Ação 3: Simulação de erro
void acao_simularErro() {
  Serial.println("\n=== SIMULACAO DE ERRO ===");
  Serial.println("[INFO] primeiro quadro corrompido propositalmente");
  Serial.println("[INFO] Receptor detecta FCS invalido e descarta");
  Serial.println("[INFO] Transmissor retransmite ate receber ACK");

  protocolo.resetarSequencia();
  protocolo.enviarCabecalho("TEXTO");
  protocolo.enviar("Erro simulado", true);   // corrompido, TX retransmite ate receber ACK
  protocolo.enviar("Recuperado OK", false);  // normal
  protocolo.enviarFim("TEXTO");
  Serial.println("=== SIMULACAO CONCLUIDA ===");
}

//  Ação 4: Demonstração completa, texto com erro + imagem
void acao_demoCompleta() {
  Serial.println("\n======================================");
  Serial.println("        DEMONSTRAÇÃO COMPLETA");
  Serial.println("======================================");

  // Fase 1: Texto (erro proposital no SEQ:2)
  Serial.println("\n=== FASE 1: TEXTO ===");
  protocolo.resetarSequencia();
  protocolo.enviarCabecalho("TEXTO");
  for (int i = 0; i < TOTAL_MSGS; i++) {
    bool simErro = (i == 2);   // Corrompe o terceiro quadro para demonstrar detecção
    protocolo.enviar(mensagensPadrao[i], simErro);
    delay(100);
  }
  protocolo.enviarFim("TEXTO");
  Serial.println("=== FIM FASE 1 ===\n");

  delay(1000);

  // Fase 2: Imagem
  Serial.println("=== FASE 2: IMAGEM ===");
  protocolo.resetarSequencia();
  protocolo.enviarCabecalho("IMAGEM");
  for (int i = 0; i < TOTAL_LINHAS; i++) {
    protocolo.enviar(imagemBitmap[i]);
    delay(100);
  }
  protocolo.enviarFim("IMAGEM");
  Serial.println("=== FIM FASE 2 ===");

  Serial.println("\n======================================");
  Serial.println("        DEMONSTRAÇÃO COMPLETA ENCERRADA");
  Serial.println("======================================");
}

//  SETUP
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("========================================");
  Serial.println("  TRANSMISSOR: Stop-and-Wait ARQ");
  Serial.println("  FCS: Checksum (soma dos bytes mod 256)");
  Serial.println("  GPIO13=TX_DADOS e GPIO27=RX_ACK");
  Serial.println("========================================");

  if (!canal.iniciar()) {
    Serial.println("[ERRO] Falha ao inicializar driver RF");
    while (true) delay(1000);
  }

  Serial.println("[OK] Driver RF inicializado");
  Serial.println("[INFO] Aguardando 3s para o receptor ficar pronto...");
  delay(3000);
}

//  LOOP
void loop() {
  menu.exibir();
  char opcao = menu.aguardarEscolha();

  switch (opcao) {
    case '1': acao_enviarTexto();    break;
    case '2': acao_enviarImagem();   break;
    case '3': acao_simularErro();    break;
    case '4': acao_demoCompleta();   break;
    default:
      Serial.println("[AVISO] Opcao invalida");
      break;
  }
}
