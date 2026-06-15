#include <RH_ASK.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define RX_PIN 14

RH_ASK driver(2000, RX_PIN, 2, 0);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(115200);

    Wire.begin(25, 26);

    lcd.init();
    lcd.backlight();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Aguardando...");

    if (!driver.init()) {
        Serial.println("Erro RF");

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Erro RF");

        while(true);
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

        lcd.clear();

        lcd.setCursor(0,0);
        lcd.print("Recebido:");

        lcd.setCursor(0,1);
        lcd.print((char*)buf);
    }
}
