/*
  ================================================================
  hardware:
  - ESP32 Receptor
  - Módulo RF 433 MHz Receptor (DATA: GPIO 14)
  - Display LCD I2C 16x2 (SDA: GPIO 25, SCL: GPIO 26)
  ================================================================
*/

// bibliotecas necessárias para o módulo RF e LCD funcionarem no ESP32
#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// pino DATA do módulo receptor RF 433 MHz
#define RX_PIN 14

// tipos de quadro aceitos pelo protocolo
#define TYPE_DATA 0x01  // quadro de dados
#define TYPE_ACK  0x02  // confirmação de recebimento
#define TYPE_END  0x03  // fim da transmissão

// cria o driver do rádio: 2000 bps, pino RX = 14, pino TX = 2 (não usado)
RH_ASK driver(2000, RX_PIN, 2, 0);

// cria o LCD: endereço 0x27, 16 colunas, 2 linhas
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*
  ================================================================
                    ESTRUTURA DO QUADRO
  ================================================================
  Cada quadro recebido possui os seguintes campos:
  FLAG  (1 byte)     - indica início do quadro (sempre 0x7E)
  TYPE  (1 byte)     - tipo do quadro (DATA, ACK ou END)
  SEQ   (1 byte)     - número de sequência (0 a 255)
  LEN   (1 byte)     - tamanho do campo DATA (0 a 16 bytes)
  DATA  (0-16 bytes) - carga útil da transmissão
  FCS   (1 byte)     - CRC-8 para detecção de erros
  ================================================================
*/

struct Quadro {
  uint8_t flag;      // início do quadro (sempre 0x7E)
  uint8_t type;      // tipo: DATA, ACK ou END
  uint8_t seq;       // número de sequência
  uint8_t len;       // tamanho dos dados
  uint8_t data[16];  // conteúdo da mensagem (máximo 16 bytes)
  uint8_t fcs;       // CRC-8 para detecção de erros
};

/*
  ================================================================
                    CÁLCULO DO CRC-8
  ================================================================
  Semelhante ao do transmissor, garante que ambos calculem
  o mesmo valor para comparação
  ================================================================
*/

uint8_t calcularCRC(const Quadro& q) {
  uint8_t crc = 0x00;

  auto processaByte = [&](uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  };

  processaByte(q.type);
  processaByte(q.seq);
  processaByte(q.len);
  for (int i = 0; i < q.len; i++)
    processaByte(q.data[i]);

  return crc;
}

/*
  ================================================================
                    VARIÁVEIS DO RECEPTOR
  ================================================================
  seqEsperado - número de sequência aguardado pelo receptor
  imagem[][] - buffer para reconstruir a imagem linha a linha
  linhasRecebidas - contador de linhas da imagem recebidas
  recebeImagem - flag que indica se está no modo imagem
  ================================================================
*/

uint8_t seqEsperado = 0; // próximo SEQ esperado
bool recebeImagem = false; // false = texto, true = imagem
char imagem[8][9]; // buffer da imagem 8x8
uint8_t linhasRecebidas = 0; // contador de linhas recebidas

/*
  ================================================================
                          SETUP
  ================================================================
  Inicializa o Monitor Serial, o display LCD e o módulo RF
  ================================================================
*/

void setup() {
  Serial.begin(115200);

  Serial.println("================================================");
  Serial.println("  RECEPTOR RF 433 MHz: ESP32");
  Serial.println("  Projeto Final: Transmissao de Dados");
  Serial.println("================================================");

  // inicia o barramento I2C nos pinos SDA=25 e SCL=26
  Wire.begin(25, 26);

  // inicia o LCD
  lcd.init();
  lcd.backlight(); // liga a luz de fundo
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aguardando...");

  // inicia o módulo RF
  if (!driver.init()) {
    Serial.println("[ERRO] Falha ao iniciar o modulo RF!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Erro no RF!");
    while (true);
  }

  Serial.println("[OK] Modulo RF iniciado com sucesso");
  Serial.println("[OK] Aguardando transmissao...");
  Serial.println("================================================");
}

/*
  ================================================================
                          LOOP
  ================================================================
  O receptor fica escutando o rádio continuamente
  Para cada quadro recebido:

  1. Reconstrói o quadro a partir dos bytes recebidos
  2. Verifica a FLAG (deve ser 0x7E)
  3. Recalcula o CRC e compara com o FCS recebido
  4. Se o quadro é válido e na ordem certa:
       - exibe no LCD e no Serial
       - envia ACK (quando tiver dois pares de módulos RF)
       - avança seqEsperado
  5. Se o quadro está fora de ordem: descarta
  6. Se o quadro é END: encerra e exibe conclusão
  ================================================================
*/

void loop() {
  uint8_t buf[32];
  uint8_t len = sizeof(buf);

  // fica escutando o rádio — retorna se não chegou nada
  if (!driver.recv(buf, &len))
    return;

  // reconstrói o quadro a partir dos bytes recebidos
  Quadro q{};
  q.flag = buf[0];
  q.type = buf[1];
  q.seq  = buf[2];
  q.len  = buf[3];
  for (int i = 0; i < q.len; i++)
    q.data[i] = buf[4 + i];
  q.fcs = buf[4 + q.len];

  // verifica FLAG — descarta quadro inválido
  if (q.flag != 0x7E) {
    Serial.println("[ERRO] FLAG invalida. Quadro descartado");
    return;
  }
  
  // verifica CRC — descarta quadro corrompido
  uint8_t crcCalculado = calcularCRC(q);
  if (crcCalculado != q.fcs) {
    Serial.print("[ERRO] CRC invalido. Esperado: 0x");
    Serial.print(crcCalculado, HEX);
    Serial.print(" | Recebido: 0x");
    Serial.println(q.fcs, HEX);
    Serial.println("[ERRO] Quadro corrompido descartado");
    return; // descarta sem enviar ACK, força retransmissao
  }

  // processa quadro de acordo com o tipo
  if (q.type == TYPE_DATA) {

    // verifica se o quadro está na ordem certa
    if (q.seq != seqEsperado) {
      Serial.print("[AVISO] Quadro fora de ordem. Esperado SEQ: ");
      Serial.print(seqEsperado);
      Serial.print(" | Recebido SEQ: ");
      Serial.println(q.seq);
      return; // descarta quadro fora de ordem
    }

    // extrai a mensagem do campo DATA
    char mensagem[17]{};
    memcpy(mensagem, q.data, q.len);
    mensagem[q.len] = '\0';

    Serial.print("[RX] SEQ: ");
    Serial.print(q.seq);
    Serial.print(" | Dados: ");
    Serial.println(mensagem);

    // modo TEXTO: exibe mensagem no LCD
    if (!recebeImagem) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SEQ:");
      lcd.print(q.seq);
      lcd.setCursor(0, 1);
      lcd.print(mensagem);

      // detecta fim do texto pela mensagem "FIM"
      if (strcmp(mensagem, "FIM") == 0) {
        Serial.println("================================================");
        Serial.println("[OK] Texto recebido! Aguardando imagem...");
        Serial.println("================================================");
        recebeImagem  = true;
        seqEsperado   = 0; // reinicia sequência para a imagem
        linhasRecebidas = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Aguard. imagem");
        return;
      }
    }

    // modo IMAGEM: armazena linha e reconstrói ao completar
    else {
      strcpy(imagem[q.seq], mensagem);
      linhasRecebidas++;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Linha:");
      lcd.print(q.seq);
      lcd.setCursor(0, 1);
      lcd.print(mensagem);

      Serial.print("[IMG] Linha ");
      Serial.print(q.seq);
      Serial.print(" armazenada: ");
      Serial.println(mensagem);

      // quando receber todas as 8 linhas, reconstrói a imagem
      if (linhasRecebidas == 8) {
        Serial.println("================================================");
        Serial.println("[OK] Imagem reconstruida:");
        for (int i = 0; i < 8; i++)
          Serial.println(imagem[i]);
        Serial.println("================================================");
      }
    }

    // avança o número de sequência esperado
    seqEsperado++;

    /*
      Aqui o receptor deveria enviar um ACK para o transmissor
      confirmando o recebimento do quadro. Com 2 pares de módulos
      RF isso funcionaria normalmente. Com 1 par, o envio do ACK
      via RF não é possível.

      Código do ACK (funcional quando tiver hardware completo):
      uint8_t ack[5] = {0x7E, TYPE_ACK, q.seq, 0, 0};
      driver.send(ack, 5);
      driver.waitPacketSent();
    */

  }

  // quadro END — fim da transmissão
  else if (q.type == TYPE_END) {
    Serial.println("[OK] Quadro END recebido. Transmissao encerrada");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Concluido!");
    lcd.setCursor(0, 1);
    lcd.print("Imagem OK");
  }
}
