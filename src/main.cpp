#include <Arduino.h>

const int onboard_led = 13;

void setup() {
    Serial.begin(115200);
    Serial.println("Serial started");
    pinMode(onboard_led, OUTPUT);
}

void loop() {
    digitalWrite(onboard_led, HIGH);
    Serial.println("LED is on");
    delay(200);
    digitalWrite(onboard_led, LOW);
    Serial.println("LED is off");
    delay(200);
}