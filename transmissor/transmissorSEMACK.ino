#include <RH_ASK.h>
#include <SPI.h>

#define TX_PIN 13

RH_ASK driver(2000, 2, TX_PIN, 0);

#define TYPE_DATA 0x01
#define TYPE_END  0x03

struct Quadro {
    uint8_t flag;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t data[16];
    uint8_t fcs;
};

uint8_t calcularCRC(const Quadro& q)
{
    uint8_t crc = 0;

    auto processaByte = [&](uint8_t byte)
    {
        crc ^= byte;

        for (int i{}; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    };

    processaByte(q.type);
    processaByte(q.seq);
    processaByte(q.len);

    for (int i{}; i < q.len; i++)
        processaByte(q.data[i]);

    return crc;
}

void enviarQuadro(const Quadro& q)
{
    uint8_t buffer[32];

    buffer[0] = q.flag;
    buffer[1] = q.type;
    buffer[2] = q.seq;
    buffer[3] = q.len;

    for (int i{}; i < q.len; i++)
        buffer[4 + i] = q.data[i];

    buffer[4 + q.len] = q.fcs;

    uint8_t tamanho = 5 + q.len;

    driver.send(buffer, tamanho);
    driver.waitPacketSent();
}

const char* mensagens[] =
{
    "OLA",
    "ESP32",
    "RF433",
    "CRC OK",
    "FIM"
};

const int totalMensagens =
    sizeof(mensagens) / sizeof(mensagens[0]);

void setup()
{
    Serial.begin(115200);

    if (!driver.init())
    {
        Serial.println("Erro RF");
        while (true);
    }

    Serial.println("TX pronto");

    delay(2000);

    for (uint8_t seq{}; seq < totalMensagens; seq++)
    {
        Quadro q{};

        q.flag = 0x7E;
        q.type = TYPE_DATA;
        q.seq = seq;

        q.len = strlen(mensagens[seq]);

        memcpy(q.data,
               mensagens[seq],
               q.len);

        q.fcs = calcularCRC(q);

        Serial.print("Enviando SEQ ");
        Serial.print(seq);
        Serial.print(" -> ");
        Serial.println(mensagens[seq]);

        enviarQuadro(q);

        delay(2000);
    }

    Quadro fim{};

    fim.flag = 0x7E;
    fim.type = TYPE_END;
    fim.seq = 0;
    fim.len = 0;
    fim.fcs = calcularCRC(fim);

    enviarQuadro(fim);

    Serial.println("Transmissao encerrada");
}

void loop()
{
}
