#include <Arduino.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

// General setup
const int ONBOARD_LED = 13;      // Onboard LED
const unsigned long WAIT = 200;  // Delay interval

// Servo motor
const int MAIN_SERVO = 4;  // D4 for main servo
const int PUSH_SERVO = 8;  // D for servo to push button

Servo main_servo;
Servo push_servo;

const int maxAngle = 30;  // Max angle of servo
const int minAngle = 0;   // Minimum angle of servo

const int pushAngle = 5;

int tightenAngle = maxAngle;
int loosenAngle = minAngle;

// Versawriter
const int VERSA_WRITER = 3;  // D3 for versawriter

// IR sensor
const int IR_SENSOR = 8;  // D8 for IR sensor

VL53L0X sensor;

const short DISTANCE_THRESHOLD = 100;

// Button
const int BLACK_BUTTON = 5;  // D5 for black button
const int WHITE_BUTTON = 6;  // D6 for white button

// Flags
bool isBeltLoosen = true;

// Error
const unsigned long SHORT_INTERVAL = 100;
const unsigned long MID_INTERVAL = 200;
const unsigned long LONG_INTERVAL = 500;

void blinkLED(unsigh long interval) {
    digitalWrite(ONBOARD_LED, HIGH);
    delay(interval);
    digitalWrite(ONBOARD_LED, LOW);
    delay(interval);
}
void sensorInitError() {
    blinkLED(SHORT_INTERVAL);
    delay(MID_INTERVAL);
}

void setup() {
    Serial.begin(9600);
    // Init onboard LED
    pinMode(ONBOARD_LED, OUTPUT);

    // Init servo motor
    main_servo.attach(MAIN_SERVO);
    push_servo.attach(PUSH_SERVO);

    // Init versawriter
    pinMode(VERSA_WRITER, OUTPUT);

    // Init IR sensor
    Wire.begin();
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1)
            sensorInitError();
    }
    sensor.startContinuous();
}

void tightenBelt() {
    main_servo.write(tightenAngle);
    Serial.println("tightenBelt");
}

void loosenBelt() {
    main_servo.write(loosenAngle);
    Serial.println("loosenAngle");
}

void pushButton() {
    push_servo.write(pushAngle);
}

void loop() {
    unsigned int distance = sensor.readRangeContinuousMillimeters();
    Serial.println("Distance: " + distance);

    // when distance is shorter than threshold, tighten belt
    if (distance < DISTANCE_THRESHOLD && isBeltLoosen) {
        tightenBelt();
        isBeltLoosen = false;
    }
    // when black button is pushed, loosen belt
    if (digitalRead(RELEASE_BUTTON) && !isBeltLoosen) {
        loosenBelt();
        isBeltLoosen = true;
    }
    if (sensor.timeoutOccurred()) {
        Serial.print("Distance: TIMEOUT");
    }
    delay(WAIT);
}