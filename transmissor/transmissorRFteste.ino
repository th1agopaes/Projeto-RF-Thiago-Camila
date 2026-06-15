#include <RH_ASK.h>
#include <SPI.h>

#define TX_PIN 13

RH_ASK driver(2000, 2, TX_PIN, 0);

void setup() {
    Serial.begin(115200);

    if (!driver.init()) {
        Serial.println("Erro ao iniciar RF");
        while (true);
    }

    Serial.println("TX pronto");
}

void loop() {
    const char* msg = "OLA";

    driver.send((uint8_t*)msg, strlen(msg));
    driver.waitPacketSent();

    Serial.println("Mensagem enviada: OLA");

    delay(2000);
}
