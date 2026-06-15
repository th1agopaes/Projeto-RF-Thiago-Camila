# Projeto Final - Comunicação via Rádio RF 433 MHz

## Autores

* Thiago
* Camila

## Disciplina

Transmissão e Comunicação de Dados

---

## Descrição do Projeto

Este projeto implementa um sistema de comunicação sem fio entre duas placas ESP32 utilizando módulos RF de 433 MHz. O sistema transmite mensagens de texto e pequenas imagens utilizando uma estrutura de quadros customizada com detecção de erros via CRC.

---

## Objetivos

* Implementar comunicação sem fio utilizando RF 433 MHz
* Desenvolver uma estrutura de quadros para transmissão dos dados
* Implementar detecção de erros através de CRC (Frame Check Sequence)
* Transmitir mensagens de texto e exibir no display LCD
* Transmitir e reconstruir pequenas imagens

> **Nota sobre o algoritmo de controle de fluxo:** A implementação do protocolo Go-Back-N ARQ requer comunicação bidirecional via RF, o que exige um módulo TX e um módulo RX em cada ESP32 (dois pares no total). Durante o desenvolvimento, identificamos que o kit disponibilizava apenas um par de módulos (1 TX + 1 RX), impossibilitando o envio de ACK via RF. Essa limitação foi confirmada pelo professor. O algoritmo Go-Back-N está documentado e implementado nos códigos, mas não pôde ser testado por restrição de hardware.

---

## Hardware Utilizado

### ESP32 Transmissor
* ESP32 Wroom Devkit v1 
* Módulo RF 433 MHz Transmissor (XLC-FST)

### ESP32 Receptor
* ESP32 Wroom Devkit v1 
* Módulo RF 433 MHz Receptor (RF-5V)
* Display LCD I2C 16x2

### Materiais Auxiliares
* Protoboard 830 pontos
* Cabos Jumpers
* Cabos USB

---

## Pinagem Utilizada

### ESP32 Transmissor

| Sinal      | GPIO    |
|------------|---------|
| RF TX DATA | GPIO 13 |

### ESP32 Receptor

| Sinal      | GPIO    |
|------------|---------|
| RF RX DATA | GPIO 14 |
| LCD SDA    | GPIO 25 |
| LCD SCL    | GPIO 26 |

---

## Arquitetura do Sistema

A transmissão de dados ocorre exclusivamente via módulos RF 433 MHz:

```
ESP32 Transmissor --> [Módulo TX RF 433 MHz] --> [Módulo RX RF 433 MHz] --> ESP32 Receptor --> LCD
```

---

## Estrutura dos Quadros

Todos os dados são encapsulados na seguinte estrutura (máximo 21 bytes):

| Campo | Tamanho   | Descrição                              |
|-------|-----------|----------------------------------------|
| FLAG  | 1 byte    | Indica início do quadro (0x7E)        |
| TYPE  | 1 byte    | Tipo do quadro (DATA, ACK ou END)     |
| SEQ   | 1 byte    | Número de sequência (0 a 255)         |
| LEN   | 1 byte    | Tamanho do campo DATA (0 a 16 bytes)  |
| DATA  | 0-16 bytes| Carga útil da transmissão             |
| FCS   | 1 byte    | CRC-8 para detecção de erros          |

### Tipos de Quadro

| Valor | Tipo | Descrição                        |
|-------|------|----------------------------------|
| 0x01  | DATA | Transmissão de dados             |
| 0x02  | ACK  | Confirmação de recebimento       |
| 0x03  | END  | Sinaliza fim da transmissão      |

---

## Detecção de Erros: CRC-8

O FCS é calculado utilizando CRC-8 com polinômio gerador 0x07, processando os campos TYPE, SEQ, LEN e DATA byte a byte:

```
Para cada byte:
  CRC = CRC XOR byte
  Para cada bit (8 vezes):
    Se bit mais significativo = 1:
      CRC = (CRC << 1) XOR 0x07
    Senão:
      CRC = CRC << 1
```

O receptor recalcula o CRC ao receber o quadro e compara com o FCS recebido. Se os valores divergem, o quadro é descartado.

---

## Simulação de Erros

O código do transmissor possui uma linha comentada que corrompe propositalmente o byte de dados do quadro de sequência 2:

```cpp
// if (q.seq == 2) q.data[0] ^= 0xFF;
```

Ao descomentar essa linha, o receptor detecta a divergência no CRC e registra no Monitor Serial:

```
ERRO CRC
```

O quadro corrompido é descartado sem ser exibido no LCD, demonstrando o funcionamento da detecção de erros.

---

## Transmissão de Texto

Mensagens de texto são encapsuladas em quadros DATA e enviadas sequencialmente. Cada mensagem é exibida no LCD ao ser recebida e validada.

Mensagens transmitidas:
* Oi
* ESP32
* RF433
* CRC
* Projeto
* Final
* FIM

---

## Transmissão de Imagens

Uma imagem bitmap 8x8 pixels é fragmentada em 8 quadros DATA, um por linha. O receptor reconstrói a imagem linha a linha e imprime no Monitor Serial ao receber todas as 8 linhas.

Imagem transmitida:

```
00111100
01000010
10011001
10100101
10100101
10011001
01000010
00111100
```

---

## Estrutura do Repositório

```
Projeto-RF-Thiago-Camila/
│
├── transmissor/
│   └── transmissor.ino        # Código principal do transmissor
│
├── receptor/
│   └── receptor.ino           # Código principal do receptor
│
├── testes/
│   ├── tx_rf_puro.ino         # Teste básico do módulo TX
│   ├── rx_rf_puro.ino         # Teste básico do módulo RX
│   ├── tx_estrutura.ino       # TX com quadro e CRC
│   ├── rx_estrutura.ino       # RX com quadro, CRC e LCD
│   ├── tx_imagem.ino          # TX transmissão de imagem
│   └── rx_imagem.ino          # RX reconstrução de imagem
│
└── README.md
```

---

## Bibliotecas Utilizadas

| Biblioteca       | Autor         | Finalidade                  |
|------------------|---------------|-----------------------------|
| RadioHead        | Mike McCauley | Comunicação RF 433 MHz      |
| SPI              | Arduino       | Dependência do RadioHead    |
| Wire             | Arduino       | Comunicação I2C com o LCD   |
| LiquidCrystal_I2C| Frank de Brabander | Controle do display LCD |

---

## Testes Realizados

1. Comunicação RF básica entre os dois ESP32
2. Transmissão com estrutura de quadro completa
3. Validação do CRC no receptor
4. Simulação de erro e detecção pelo receptor
5. Exibição de mensagens no display LCD
6. Transmissão e reconstrução de imagem bitmap 8x8
