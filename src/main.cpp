#include <Arduino.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

#define UPLOAD_SPEED 115200

// 300/ 1200/ 2400/ 4800/ 9600/ 14400/ 19200/ 28800/ 38400/ 57600/ 115200

// General setup
#define ONBOARD_LED 13  // Onboard LED
#define WAIT 200        // Delay interval

// Servo motor
#define MAIN_SERVO 4  // D4 for main servo
#define PUSH_SERVO 2  // D2 for servo to push button

Servo main_servo;
Servo push_servo;

#define maxAngle 90   // Max angle of main servo
#define minAngle 180  // Minimum angle of servo

#define pushAngle 1

int tightenAngle = maxAngle;
int loosenAngle = minAngle;

// Versawriter
#define VERSA_WRITER 3  // D3 for versawriter

// IR sensor
#define IR_SENSOR 8  // D8 for IR sensor

VL53L0X sensor;

#define DISTANCE_THRESHOLD 100

// Button
#define BLACK_BUTTON 6  // D6 for black button
#define WHITE_BUTTON 5  // D5 for white button

// Flags
bool isBeltLoosen = true;

void setup() {
    Serial.begin(UPLOAD_SPEED);
    Serial.println("Serial started");

    // Init onboard LED
    // pinMode(ONBOARD_LED, OUTPUT);
    // digitalWrite(ONBOARD_LED, LOW);

    // Init servo motor
    main_servo.attach(MAIN_SERVO);
    push_servo.attach(PUSH_SERVO);

    // Init versawriter
    pinMode(VERSA_WRITER, OUTPUT);

    // Init buttons
    pinMode(BLACK_BUTTON, INPUT);
    pinMode(WHITE_BUTTON, INPUT);

    // Init IR sensor
    Wire.begin();
    sensor.setTimeout(500);
    sensor.init();

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {
        }
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
    push_servo.write(4);
    delay(10);
    push_servo.write(12);
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
    if (digitalRead(BLACK_BUTTON) && !isBeltLoosen) {
        loosenBelt();
        pushButton();
        isBeltLoosen = true;
    }
    if (sensor.timeoutOccurred()) {
        Serial.print("Distance: TIMEOUT");
    }
    delay(WAIT);
}
