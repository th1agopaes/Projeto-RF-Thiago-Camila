# Projeto Final - Comunicação via Rádio RF 433 MHz

## Autores

* Thiago Paes
* Camila

## Disciplina

Transmissão e Comunicação de Dados

## Descrição do Projeto

Este projeto implementa um sistema de comunicação sem fio entre duas placas ESP32 utilizando módulos RF de 433 MHz. O sistema permite a transmissão confiável de mensagens de texto e pequenas imagens.

A solução implementa mecanismos de enquadramento, detecção de erros, controle de fluxo e reconstrução dos dados recebidos, simulando conceitos presentes em protocolos reais de comunicação de dados.

---

# Objetivos

* Implementar comunicação sem fio utilizando RF 433 MHz;
* Desenvolver uma estrutura de quadros para transmissão dos dados;
* Implementar detecção de erros através de CRC (Frame Check Sequence);
* Implementar algoritmo de controle de fluxo Go-Back-N ARQ;
* Permitir transmissão confiável de texto;
* Permitir transmissão e reconstrução de pequenas imagens;
* Exibir os dados recebidos em um display LCD.

---

# Hardware Utilizado

## Transmissor

* ESP32
* Módulo RF 433 MHz XLC-FST (Transmissor)
* Módulo RF 433 MHz RF-5V (Receptor)

## Receptor

* ESP32
* Módulo RF 433 MHz XLC-FST (Transmissor)
* Módulo RF 433 MHz RF-5V (Receptor)
* Display LCD I2C 16x2

## Materiais Auxiliares

* Protoboard
* Cabos Jumpers
* Cabos USB

---

# Arquitetura do Sistema

A comunicação ocorre exclusivamente através dos módulos RF de 433 MHz.

Fluxo de comunicação:

ESP32 TX -> RF -> ESP32 RX

Para confirmação de recebimento:

ESP32 RX -> RF -> ESP32 TX

O sistema opera de forma bidirecional, permitindo a transmissão de quadros de dados e quadros de confirmação (ACK).

---

# Estrutura dos Quadros

Todos os dados são transmitidos utilizando a seguinte estrutura:

| Campo | Tamanho  |
| ----- | -------- |
| FLAG  | 1 byte   |
| TYPE  | 1 byte   |
| SEQ   | 1 byte   |
| LEN   | 1 byte   |
| DATA  | Variável |
| FCS   | 1 byte   |

Descrição dos campos:

* FLAG: indica início do quadro;
* TYPE: identifica o tipo do quadro;
* SEQ: número de sequência;
* LEN: tamanho dos dados;
* DATA: carga útil;
* FCS: código de verificação de integridade.

---

# Tipos de Quadros

## DATA

Responsável pela transmissão dos dados.

## ACK

Responsável pela confirmação de recebimento.

## END

Indica o término da transmissão.

---

# Detecção de Erros

Foi implementado um Frame Check Sequence (FCS) utilizando CRC.

O CRC é calculado sobre:

* TYPE
* SEQ
* LEN
* DATA

Ao receber um quadro, o receptor recalcula o CRC, o valor calculado é comparado ao FCS recebido, os quadros inválidos são descartados e os quadros válidos são processados

---

# Controle de Fluxo

O projeto implementa o protocolo Go-Back-N ARQ.

## Funcionamento

1. O transmissor envia uma janela de quadros.
2. O receptor verifica cada quadro recebido.
3. Quadros válidos geram ACK.
4. Caso um ACK não seja recebido dentro do tempo limite:

   * ocorre timeout;
   * os quadros pendentes são retransmitidos.
5. A transmissão continua após a confirmação dos quadros enviados.

## Benefícios

* Recuperação automática de erros;
* Garantia de entrega dos dados;
* Melhor utilização do canal de comunicação.

---

# Simulação de Erros

Para validar a robustez do sistema foram introduzidos erros durante a transmissão.

Foram observados:

* Detecção de quadros corrompidos;
* Descarte de mensagens inválidas;
* Retransmissão automática;
* Recuperação correta dos dados.

---

# Transmissão de Texto

Mensagens de texto são encapsuladas em quadros DATA e enviadas ao receptor.

Após a validação do CRC, o conteúdo é exibido no display LCD e registrado no monitor serial.

---

# Transmissão de Imagens

Pequenas imagens são fragmentadas em múltiplos quadros.

Cada linha da imagem é enviada em um quadro independente contendo:

* número de sequência;
* conteúdo da linha;
* CRC.

Após o recebimento completo, a imagem é reconstruída pelo receptor.

Exemplo:

00111100
01000010
10011001
10100101
10100101
10011001
01000010
00111100

---

# Testes Realizados

Foram realizados os seguintes testes:

1. Transmissão de mensagens de texto;
2. Recepção e exibição no LCD;
3. Validação do CRC;
4. Simulação de erros em quadros;
5. Recuperação por retransmissão;
6. Transmissão de imagem fragmentada;
7. Reconstrução da imagem no receptor;
8. Testes de confirmação utilizando quadros ACK.
9. Testes do algoritmo Go-Back-N ARQ.

# Resultados Obtidos

O sistema foi capaz de:

* Transmitir mensagens de texto via RF;
* Exibir dados recebidos em LCD;
* Detectar erros utilizando CRC;
* Realizar o controle de fluxo com Go-Back-N ARQ;
* Recuperar dados através de retransmissões;
* Reconstruir imagens fragmentadas.

---

# Estrutura do Repositório

```text
Projeto-RF-Thiago-Camila/
│
├── transmissor/
│   └── transmissor.ino
│
├── receptor/
│   └── receptor.ino
│
├── docs/
│   ├── relatorio.pdf
│   └── imagens/
│
└── README.md
```

---

# Bibliotecas Utilizadas

* RadioHead
* Wire
* LiquidCrystal_I2C
* SPI

---

# Demonstração

Durante a demonstração são apresentados:

* Comunicação RF entre os ESP32;
* Exibição das mensagens no LCD;
* Logs da transmissão;
* Detecção de erros;
* Recuperação por retransmissão;
* Reconstrução de imagem.

---
# Pinagem Utilizada

## ESP32 Transmissor

| Sinal | GPIO |
|---------|---------|
| RF TX DATA | GPIO 13 |
| RF RX DATA | x |

## ESP32 Receptor

| Sinal | GPIO |
|---------|---------|
| RF TX DATA | GPIO 14 |
| RF RX DATA | y |

## LCD I2C

| Sinal | GPIO |
|---------|---------|
| SDA | GPIO 25 |
| SCL | GPIO 26 |
