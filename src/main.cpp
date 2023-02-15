#include <Arduino.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SERVO_PIN 4        // D4 Pin
#define SENSOR_PIN 8       // D8 Pin
#define ONBOARD_LED 13     // Onboard LED
#define DECREASE_BUTTON 5  // D5 Pin, black button
#define INCREASE_BUTTON 6  // D6 Pin, white button

Servo servo;
VL53L0X sensor;

const unsigned long WAIT = 200;

const int maxAngle = 90;
const int minAngle = -90;

int tightenAngle = maxAngle;
int loosenAngle = minAngle;

void init_sensor() {
    Wire.begin();
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {
        }
    }
    sensor.startContinuous();
}

void setup() {
    // Initialize on-board LED
    pinMode(ONBOARD_LED, OUTPUT);
    // Initialize servo
    servo.attach(SERVO_PIN);

    // Initialize adjust buttons
    pinMode(DECREASE_BUTTON, INPUT);
    pinMode(INCREASE_BUTTON, INPUT);

    // Initialize distance sensor
    init_sensor();

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
    /* if (digitalRead(DECREASE_BUTTON) == LOW)
        decreaseAngle();

    if (digitalRead(INCREASE_BUTTON) == LOW)
        increaseAngle();

    tightenBelt();
    loosenBelt(); */
    unsigned int distance = sensor.readRangeContinuousMillimeters();
    Serial.print(distance);

    if (distance < 100)
        digitalWrite(ONBOARD_LED, HIGH);
    else
        digitalWrite(ONBOARD_LED, LOW);

    if (sensor.timeoutOccurred())
        Serial.print(" TIMEOUT");

    Serial.println();

    delay(100);
}
