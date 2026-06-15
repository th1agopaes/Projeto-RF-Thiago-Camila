#include <RH_ASK.h>
#include <SPI.h>

#define RX_PIN 14

RH_ASK driver(2000, RX_PIN, 2, 0);

void setup() {
    Serial.begin(115200);

    if (!driver.init()) {
        Serial.println("Erro ao iniciar RF");
        while (true);
    }

    Serial.println("RX pronto");
}

void loop() {
    uint8_t buf[50];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) {
        buf[buflen] = '\0';

        Serial.print("Recebido: ");
        Serial.println((char*)buf);
    }
}
