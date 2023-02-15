#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN 4        // D4 Pin
#define DECREASE_BUTTON 5  // D5 Pin, black button
#define INCREASE_BUTTON 6  // D6 Pin, white button

Servo servo;

const unsigned long WAIT = 200;

const int maxAngle = 90;
const int minAngle = -90;

int tightenAngle = maxAngle;
int loosenAngle = minAngle;

void setup() {
    // put your setup code here, to run once:
    servo.attach(SERVO_PIN);

    pinMode(DECREASE_BUTTON, INPUT);
    pinMode(INCREASE_BUTTON, INPUT);

    Serial.begin(9600);
}

void tightenBelt() {
    servo.write(tightenAngle);
    delay(WAIT);
}

void loosenBelt() {
    servo.write(loosenAngle);
    delay(WAIT);
}

void decreaseAngle() {
    if (tightenAngle > minAngle + 5)
        tightenAngle -= 5;

    Serial.println("Decrease angle");
}

void increaseAngle() {
    if (tightenAngle < maxAngle - 5)
        tightenAngle += 5;

    Serial.println("Increase angle");
}

void loop() {
    // put your main code here, to run repeatedly:
    if (digitalRead(DECREASE_BUTTON) == LOW)
        decreaseAngle();

    if (digitalRead(INCREASE_BUTTON) == LOW)
        increaseAngle();

    tightenBelt();
    loosenBelt();
}
