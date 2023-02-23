#include <Arduino.h>
#include <Servo.h>

const unsigned long wait = 1000;

const int onboard_led = 13;

const int black_button = 6;
const int white_button = 5;

const int main_servo_pin = 4;
const int push_servo_pin = 2;

Servo main_servo;
Servo push_servo;

bool isBeltLoosen = true;

int currentAngle = 90;

void setup() {
    Serial.begin(115200);
    Serial.println("Serial started");

    pinMode(onboard_led, OUTPUT);

    pinMode(black_button, INPUT);
    pinMode(white_button, INPUT);

    main_servo.attach(main_servo_pin);
    push_servo.attach(push_servo_pin);
}

void loop() {
    if (!digitalRead(black_button)) {
        if (isBeltLoosen) {
            currentAngle = 90;
            isBeltLoosen = false;
        } else {
            currentAngle = 180;
            isBeltLoosen = true;
        }
    }
    if (!digitalRead(white_button)) {
        push_servo.write(0);
    } else {
        push_servo.write(20);
    }
    main_servo.write(currentAngle);
    Serial.print("Now: ");
    Serial.println(currentAngle);

    delay(100);
}