#include "arm.h"

Servo32U4Pin5 servo;

#define AMP_PIN 12
#define RAISED_PWM 570
#define LOWERED_PWM 1700
void Arm::enterInit()
{
    servo.attach();
    servo.setTargetPos(RAISED_PWM);
    EnterIdle();
}

void Arm::update()
{
    servo.update();
    switch (state) {
        case IDLE:
            HandleIdle(); break;
        case LOWERING:
            HandleLowering(); break;
        case RAISING:
            HandleRaising(); break;
        case 
        
    }
}

void Arm::EnterIdle()
{
    state = IDLE;
}

void Arm::HandleIdle()
{
    if (checkLowerCmd()) {
        EnterLowering();
    }
    if (checkRaiseCmd()) {
        EnterRaising();
    }
    if (checkWeighCmd()) {
        EnterWeighing();
    }
}

void Arm::EnterRaising()
{
    servo.setTargetPos(RAISED_PWM);
    state = RAISING;
}

void Arm::HandleRaising()
{
    if (servo.checkAtTarget()) {
        EnterIdle();
    }
}

void Arm::EnterLowering()
{
    servo.setTargetPos(LOWERED_PWM);
    state = LOWERING;
}

void Arm::HandleLowering()
{
    if (servo.checkAtTarget()) {
        EnterIdle();
    }
}

void Arm::EnterWeighing()
{
}

void Arm::HandleWeighing()
{
}
