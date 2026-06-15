#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define RX_PIN 14

RH_ASK driver(2000, RX_PIN, 2, 0);

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define TYPE_DATA 0x01
#define TYPE_END  0x03

struct Quadro
{
    uint8_t flag;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t data[16];
    uint8_t fcs;
};

char imagem[8][9];
uint8_t linhasRecebidas = 0;

uint8_t calcularCRC(const Quadro& q)
{
    uint8_t crc = 0;

    auto processaByte = [&](uint8_t byte)
    {
        crc ^= byte;

        for (int i = 0; i < 8; i++)
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

    for (int i = 0; i < q.len; i++)
        processaByte(q.data[i]);

    return crc;
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(25, 26);

    lcd.init();
    lcd.backlight();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Aguardando...");

    if (!driver.init())
    {
        Serial.println("Erro RF");
        while (true);
    }

    Serial.println("RX pronto");
}

void loop()
{
    uint8_t buf[32];
    uint8_t len = sizeof(buf);

    if (!driver.recv(buf, &len))
        return;

    Quadro q{};

    q.flag = buf[0];
    q.type = buf[1];
    q.seq = buf[2];
    q.len = buf[3];

    for (int i = 0; i < q.len; i++)
        q.data[i] = buf[4 + i];

    q.fcs = buf[4 + q.len];

    if (q.flag != 0x7E)
    {
        Serial.println("FLAG invalida");
        return;
    }

    uint8_t crc = calcularCRC(q);

    if (crc != q.fcs)
    {
        Serial.println("ERRO CRC");
        return;
    }

    if (q.type == TYPE_DATA)
    {
        char mensagem[17];

        memcpy(mensagem,
               q.data,
               q.len);

        mensagem[q.len] = '\0';

        Serial.print("Linha ");
        Serial.print(q.seq);
        Serial.print(" -> ");
        Serial.println(mensagem);

        strcpy(imagem[q.seq], mensagem);

        linhasRecebidas++;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Linha:");
        lcd.print(q.seq);

        lcd.setCursor(0, 1);
        lcd.print(mensagem);

        if (linhasRecebidas == 8)
        {
            Serial.println();
            Serial.println("Imagem reconstruida:");

            for (int i = 0; i < 8; i++)
            {
                Serial.println(imagem[i]);
            }

            linhasRecebidas = 0;
        }
    }

    if (q.type == TYPE_END)
    {
        Serial.println();
        Serial.println("Fim da transmissao");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Imagem OK");
        lcd.setCursor(0, 1);
        lcd.print("Concluido");
    }
}
