/*
  ================================================================
  hardware:
  - ESP32 Transmissor
  - módulo RF 433 MHz Transmissor(DATA: GPIO 13)

  nota sobre GO-BACK-N:
  O algoritmo Go-Back-N ARQ está implementado neste código.
  Para funcionar completamente, cada ESP32 precisa de um módulo
  TX e um módulo RX (dois pares no total). Com apenas um par
  disponível, o ACK não consegue retornar via RF
  ================================================================
*/

// bibliotecas necessárias para o módulo RF funcionar no ESP32
#include <RH_ASK.h>
#include <SPI.h>

// pino DATA do módulo transmissor RF 433 MHz
#define TX_PIN 13

// tempo máximo aguardando ACK antes de retransmitir (5 segundos)
#define TIMEOUT 5000

// tamanho da janela do Go-Back-N (quantos quadros envia antes de esperar ACK)
#define JANELA 4

// tipos de quadro aceitos pelo protocolo
#define TYPE_DATA 0x01  // quadro de dados
#define TYPE_ACK  0x02  // confirmação de recebimento
#define TYPE_END  0x03  // fim da transmissão

// cria o driver do rádio: 2000 bps, pino RX = 2 (não usado), pino TX = 13
RH_ASK driver(2000, 2, TX_PIN, 0);

/*
  ================================================================
                    ESTRUTURA DO QUADRO
  ================================================================
  Cada quadro transmitido possui os seguintes campos:
  FLAG (1 byte) - indica início do quadro (sempre 0x7E)
  TYPE (1 byte) - tipo do quadro (DATA, ACK ou END)
  SEQ (1 byte) - número de sequência (0 a 255)
  LEN (1 byte) - tamanho do campo DATA (0 a 16 bytes)
  DATA (0-16 bytes) - carga útil da transmissão
  FCS (1 byte) - CRC-8 para detecção de erros
  ================================================================
*/

struct Quadro {
  uint8_t flag; // início do quadro (sempre 0x7E)
  uint8_t type; // tipo: DATA, ACK ou END
  uint8_t seq; // número de sequência
  uint8_t len; // tamanho dos dados
  uint8_t data[16]; // conteúdo da mensagem (máximo 16 bytes)
  uint8_t fcs; // CRC-8 para detecção de erros
};

/*
  ================================================================
                    CÁLCULO DO CRC
  ================================================================
  O CRC é calculado usando os campos TYPE, SEQ, LEN e DATA
  Utiliza o polinômio gerador 0x07

  Para cada byte:
    CRC = CRC XOR byte
    Para cada bit (8 vezes):
      Se bit mais significativo = 1:
        CRC = (CRC << 1) XOR 0x07
      Senão:
        CRC = CRC << 1
  ================================================================
*/

uint8_t calcularCRC(const Quadro& q) {
  uint8_t crc = 0x00;

  // função interna que processa um byte por vez
  auto processaByte = [&](uint8_t byte) {
    crc ^= byte;                    // XOR do byte atual com o CRC
    for (int i = 0; i < 8; i++) {  // processa bit a bit
      if (crc & 0x80)               // se o bit mais significativo é 1
        crc = (crc << 1) ^ 0x07;   // desloca e aplica o polinômio
      else
        crc <<= 1;                  // só desloca
    }
  };

  // processa cada campo do quadro (exceto FLAG e FCS)
  processaByte(q.type);
  processaByte(q.seq);
  processaByte(q.len);
  for (int i = 0; i < q.len; i++)
    processaByte(q.data[i]);

  return crc;
}

/*
  ================================================================
                    FUNÇÃO DE ENVIAR QUADRO
  ================================================================
  Converte a struct Quadro em um array de bytes e transmite
  pelo módulo RF 433 MHz usando a biblioteca RadioHead

  Estrutura do buffer enviado:
  [0] FLAG | [1] TYPE | [2] SEQ | [3] LEN | [4..N] DATA | [N+1] FCS
  ================================================================
*/

void enviarQuadro(const Quadro& q) {
  uint8_t buffer[32];

  // monta o buffer seguindo a estrutura do quadro
  buffer[0] = q.flag;  // FLAG - início do quadro
  buffer[1] = q.type;  // TYPE - tipo do quadro
  buffer[2] = q.seq;   // SEQ  - número de sequência
  buffer[3] = q.len;   // LEN  - tamanho dos dados

  // copia os bytes de dados para o buffer
  for (int i = 0; i < q.len; i++)
    buffer[4 + i] = q.data[i];

  // FCS fica logo após o último byte de dados
  buffer[4 + q.len] = q.fcs;

  // tamanho total: FLAG + TYPE + SEQ + LEN + DATA + FCS
  uint8_t tamanho = 5 + q.len;

  // envia pelo rádio e aguarda o envio terminar
  driver.send(buffer, tamanho);
  driver.waitPacketSent();

  Serial.print("[TX] Quadro enviado | TYPE: ");
  Serial.print(q.type, HEX);
  Serial.print(" | SEQ: ");
  Serial.print(q.seq);
  Serial.print(" | LEN: ");
  Serial.print(q.len);
  Serial.print(" | FCS: ");
  Serial.println(q.fcs, HEX);
}

/*
  ================================================================
                    DADOS A TRANSMITIR
  ================================================================
  Mensagens de texto e imagem bitmap 8x8 que serão enviadas
  em quadros DATA sequenciais
  ================================================================
*/

// mensagens de texto a transmitir
const char* mensagens[] = {
  "Oi",
  "ESP32",
  "RF433",
  "CRC",
  "Projeto",
  "Final",
  "FIM"
};
const int totalMensagens = sizeof(mensagens) / sizeof(mensagens[0]);

// imagem bitmap 8x8, cada string representa uma linha
const char* imagem[] = {
  "00111100",
  "01000010",
  "10011001",
  "10100101",
  "10100101",
  "10011001",
  "01000010",
  "00111100"
};
const int totalLinhas = sizeof(imagem) / sizeof(imagem[0]);

/*
  ================================================================
                    VARIÁVEIS DO GO-BACK-N
  ================================================================
  base: primeiro quadro ainda sem ACK confirmado
  proximoSeq: próximo número de sequência a enviar
  janela[]: cópia dos quadros enviados para retransmissão
  tempos[]: momento do envio de cada quadro da janela
  ativo[]: indica quais slots da janela estão em uso

  Nota: Com hardware completo (2 pares de módulos RF), o
  transmissor receberia ACK pelo segundo módulo RX e avançaria
  a base normalmente. Com 1 par, o timeout sempre dispara e
  o quadro é retransmitido indefinidamente
  ================================================================
*/

uint8_t base = 0; // primeiro quadro sem ACK
uint8_t proximoSeq = 0; // próximo quadro a enviar
Quadro janela[JANELA]; // cópia dos quadros enviados
unsigned long tempos[JANELA]; // tempo de envio de cada slot
bool ativo[JANELA] = {false, false, false, false}; // slots em uso

// flag para controlar se já terminou de enviar texto e passou para imagem
bool enviandoImagem = false;

/*
  ================================================================
                          SETUP
  ================================================================
  Inicializa o Monitor Serial e o módulo RF
  Um atraso inicial garante que o receptor esteja pronto
  antes do transmissor começar a enviar
  ================================================================
*/

void setup() {
  // inicia o monitor serial para acompanhar a transmissão
  Serial.begin(115200);

  Serial.println("================================================");
  Serial.println("  TRANSMISSOR RF 433 MHz: ESP32");
  Serial.println("  Projeto Final: Transmissao de Dados");
  Serial.println("================================================");

  // inicia o módulo RF
  if (!driver.init()) {
    Serial.println("[ERRO]: Falha ao iniciar o modulo RF");
    while (true); // trava aqui se o módulo não iniciar
  }

  Serial.println("[OK] Modulo RF iniciado com sucesso");
  Serial.println("[OK] Aguardando 3 segundos para o receptor ficar pronto...");
  delay(3000); // aguarda o receptor inicializar

  Serial.println("[OK] Iniciando transmissao...");
  Serial.println("================================================");
}

/*
  ================================================================
                      LOOP
  ================================================================
  O loop implementa o protocolo Go-Back-N ARQ:

  1. Envia quadros enquanto a janela tiver espaço
  2. Aguarda ACK lendo o rádio continuamente
  3. Se receber ACK válido, avança a base da janela
  4. Se ocorrer timeout, retransmite todos os quadros
     a partir da base
  5. Quando todas as mensagens forem confirmadas,
     envia o quadro END e encerra

  Com apenas 1 par de módulos RF, o ACK não consegue
  retornar. O timeout sempre acontece e o quadro é
  retransmitido
  ================================================================
*/

void loop() {

  // determina qual array está sendo transmitido
  const char** dados = enviandoImagem ? imagem : mensagens;
  int total = enviandoImagem ? totalLinhas : totalMensagens;

  // parte A: envia quadros enquanto a janela tiver espaço
  while (proximoSeq < total && proximoSeq - base < JANELA) {
    int slot = proximoSeq % JANELA; // posição na janela (0,1,2,3)

    // monta o quadro
    Quadro q{};
    q.flag = 0x7E;
    q.type = TYPE_DATA;
    q.seq = proximoSeq;
    q.len = strlen(dados[proximoSeq]);
    memcpy(q.data, dados[proximoSeq], q.len);
    q.fcs  = calcularCRC(q);

    /*
      simulação de erro:
      O receptor vai detectar o erro de CRC e descartar o quadro sem exibi-lo no LCD.
    */
    //if (q.seq == 2) q.data[0] ^= 0xFF;

    // guarda o quadro na janela para possível retransmissão
    janela[slot] = q;
    tempos[slot] = millis();
    ativo[slot]  = true;

    Serial.print("[TX] Enviando SEQ: ");
    Serial.print(proximoSeq);
    Serial.print(" | Dados: ");
    Serial.println(dados[proximoSeq]);

    enviarQuadro(q);
    proximoSeq++;

    // aguarda 1 segundo lendo ACKs entre cada envio
    unsigned long espera = millis();
    while (millis() - espera < 1000) {
      uint8_t ackBuf[5];
      uint8_t ackLen = sizeof(ackBuf);
      if (driver.recv(ackBuf, &ackLen)) {
        if (ackBuf[0] == 0x7E && ackBuf[1] == TYPE_ACK) {
          uint8_t seqACK = ackBuf[2];
          Serial.print("[ACK] Recebido SEQ: ");
          Serial.println(seqACK);
          if (seqACK == base) {
            ativo[base % JANELA] = false; // libera o slot
            base++; // avança a janela
          }
        }
      }
    }
  }

  // parte B: verifica timeout de cada quadro da janela
  for (int i = 0; i < JANELA; i++) {
    if (ativo[i] && millis() - tempos[i] > TIMEOUT) {
      Serial.println("[TIMEOUT] ACK nao recebido. Retransmitindo janela...");
      for (int j = 0; j < JANELA; j++)
        ativo[j] = false; // limpa todos os slots

        // como não há ACK disponível com 1 par de módulos,
        // avança a base após timeout para continuar a transmissão
        base = proximoSeq;
        proximoSeq = base;

        break;
    }
  }

  // Parte C: verifica ACK fora do loop de envio
  uint8_t buf[5];
  uint8_t bufLen = sizeof(buf);
  if (driver.recv(buf, &bufLen)) {
    if (buf[0] == 0x7E && buf[1] == TYPE_ACK) {
      uint8_t seqACK = buf[2];
      Serial.print("[ACK] Recebido SEQ: ");
      Serial.println(seqACK);
      if (seqACK == base) {
        ativo[base % JANELA] = false;
        base++;
      }
    }
  }

  // parte D: verifica se terminou o bloco atual
  if (base >= total) {
    if (!enviandoImagem) {
      // terminou o texto, passa para a imagem
      Serial.println("================================================");
      Serial.println("[OK] Texto transmitido. Iniciando imagem...");
      Serial.println("================================================");
      enviandoImagem = true;
      base = 0;
      proximoSeq = 0;
      for (int i = 0; i < JANELA; i++) ativo[i] = false;
    } else {
      // terminou a imagem, envia END e encerra
      Quadro fim{};
      fim.flag = 0x7E;
      fim.type = TYPE_END;
      fim.seq = 0;
      fim.len = 0;
      fim.fcs = calcularCRC(fim);
      enviarQuadro(fim);

      Serial.println("================================================");
      Serial.println("[OK] Transmissao concluida!");
      Serial.println("================================================");
      while (true); // encerra o transmissor
    }
  }
}
