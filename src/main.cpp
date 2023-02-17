#include <Arduino.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

const int SERVO_PIN = 4;        // D4 Pin for Servo Motor
const int SENSOR_PIN = 8;       // D8 Pin for IR Sensor
const int ONBOARD_LED = 13;     // Onboard LED for Debugging
const int RELEASE_BUTTON = 5;   // D5 Pin, black button
const int INCREASE_BUTTON = 6;  // D6 Pin, white button
const int CLOCK_PIN = 2;        // D3 for Clock

const short distance_threshold = 100;  // 腕の接近距離判定

const unsigned long WAIT = 200;

const int maxAngle = 180;  // サーボモータの最大角度
const int minAngle = 0;    // サーボモータの最小角度

int tightenAngle = maxAngle;  // ベルトを締めるときのサーボモータの角度
int loosenAngle = minAngle;  // ベルトを緩めるときのサーボモータの角度

bool isBeltTighten = false;  // ベルトは締まっているか？

Servo servo;
VL53L0X sensor;

void initSensor() {
    Wire.begin();
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {
            digitalWrite(ONBOARD_LED, HIGH);
            delay(200);
            digitalWrite(ONBOARD_LED, LOW);
            delay(200);
        }
    }
    sensor.startContinuous();
}

void setup() {
    // Initialize on-board LED
    pinMode(ONBOARD_LED, OUTPUT);
    // Initialize servo
    servo.attach(SERVO_PIN);

    pinMode(CLOCK_PIN, OUTPUT);

    // Initialize buttons
    pinMode(RELEASE_BUTTON, INPUT);
    pinMode(INCREASE_BUTTON, INPUT);

    // Initialize distance sensor
    initSensor();

    Serial.begin(9600);
}

// ベルトの制御
// 距離センサの値が一定値以下の場合はベルトを締める
// 黒いボタンを押すとベルトを緩める
void controlBelt() {
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    Serial.println(distance);
}

void catchArm() {
    servo.write(tightenAngle);
    Serial.println("Catch arm");
    delay(WAIT);
}

void releaseArm() {
    servo.write(loosenAngle);
    delay(WAIT);
}

void decreaseAngle() {
    tightenAngle -= tightenAngle > minAngle + 5 ? 5 : 0;

    Serial.println("Decrease angle");
}

void increaseAngle() {
    tightenAngle += tightenAngle < maxAngle - 5 ? 5 : 0;

    Serial.println("Increase angle");
}

void pushButton() {
    pushServo.write(30);
}

void loop() {
    unsigned short distance = sensor.readRangeContinuousMillimeters();
    Serial.print(distance);

    // ベルトを締める
    if (distance < DISTANCE_THRESHOLD && isBeltLoosen) {
        tightenBelt();
        pushButton();
        isBeltLoosen = false;
    }
    // ベルトを緩める
    if (digitalRead(RELEASE_BUTTON) && !isBeltLoosen) {
        loosenBelt();
        pushButton();
        isBeltLoosen = true;
    }
    if (sensor.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
    }
    Serial.println();

    delay(100);
}