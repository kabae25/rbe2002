#include <arm.h>

Servo32U4Pin5 servo;

#define AMP_PIN 10

void Arm::init() {
    servo.attach();
    pinMode(AMP_PIN, INPUT);
}

void Arm::update() {
    servo.update();
}

bool Arm::raise() {
    Serial.println("Arm Raising");
    utilSetLifter(1000); // 800 works
    if (servo.currentPos == 1000) {
        return true;
    }
    else return false;
}

bool Arm::lower() {
    Serial.println("Arm Lowering");
    servo.setTargetPos(2000);
    if (servo.currentPos == 2000) {
        return true;
    }
    else return false;
}

void Arm::utilSetLifter(uint16_t pulseLengthUS) {
    servo.setTargetPos(pulseLengthUS);
}

int Arm::measureWeight() {
    return analogRead(AMP_PIN);
}