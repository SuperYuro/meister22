#include <Arduino.h>
#include <Servo.h>

const unsigned long wait = 1000;

const int onboard_led = 13;

const int black_button = 6;
const int white_button = 5;

const int main_servo = 4;
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
        servo.write(180);
        Serial.println("Now: 180");
    }
    if (!digitalRead(black_button)) {
        servo.write(90);
        Serial.println("Now: 90");
    }
    delay(200);
}