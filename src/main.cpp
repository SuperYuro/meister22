#include <Arduino.h>
#include <Servo.h>

const unsigned long wait = 1000;

const int onboard_led = 13;

const int black_button = 6;
const int white_button = 5;

const int main_servo = 2;
Servo servo;

bool flag = true;

int angle = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Serial started");

    pinMode(onboard_led, OUTPUT);

    pinMode(black_button, INPUT);
    pinMode(white_button, INPUT);

    servo.attach(main_servo);
}

void loop() {
    if (!digitalRead(white_button)) {
        servo.write(4);
        Serial.println("Now: pushing");
        delay(100);
        servo.write(12);
        Serial.println("Now: releasing");
    }
    delay(100);
}