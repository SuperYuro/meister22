#include <Arduino.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

#define UPLOAD_SPEED 115200

// 300/ 1200/ 2400/ 4800/ 9600/ 14400/ 19200/ 28800/ 38400/ 57600/ 115200

// General setup
#define ONBOARD_LED 13  // Onboard LED
#define WAIT 200        // Delay interval

#define MAIN_SERVO 4  // D4 for main servo

Servo main_servo;

// IR sensor
// #define IR_SENSOR 8  // D8 for IR sensor

VL53L0X sensor;

#define DISTANCE_THRESHOLD 100

// Button
#define BLACK_BUTTON 6  // D6 for black button
#define WHITE_BUTTON 5  // D5 for white button

void setup() {
    Serial.begin(UPLOAD_SPEED);
    Serial.println("Serial started");

    // Init onboard LED
    pinMode(ONBOARD_LED, OUTPUT);

    // Init servo motor
    main_servo.attach(MAIN_SERVO);

    // Init buttons
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

void loop() {
    unsigned int distance = sensor.readRangeContinuousMillimeters();
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance < DISTANCE_THRESHOLD) {
        digitalWrite(ONBOARD_LED, HIGH);

        main_servo.write(180);

    } else {
        digitalWrite(ONBOARD_LED, LOW);
        main_servo.write(90);
    }
    if (!digitalRead(WHITE_BUTTON)) {
        main_servo.write(90);
    }

    delay(WAIT);
}
