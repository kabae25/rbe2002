#include "arm.h"

#include "constants.h"

Servo32U4Pin5 servo;



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
        case WEIGHING:
            HandleWeighing(); break;
    }
}

bool Arm::checkArmRaised()
{
    if (servo.checkAtTarget() && raiseNeedsToBeChecked) {
        raiseNeedsToBeChecked = false;
        return true;
    }
    else return false;
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

// Returns TRUE if arm is busy weighing
bool Arm::checkWeighingComplete()
{
    return isDoneWeighing;
}

void Arm::EnterRaising()
{
    servo.setTargetPos(RAISED_PWM);
    state = RAISING;
}

void Arm::HandleRaising()
{
    if (servo.checkAtTarget()) {
        EnterWeighing();
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
    weightTimer.start(50);
    state = WEIGHING;
    weightSum = 0;
    weight_count = 0;
    //Serial.println("Entering Weighing");
}

float Arm::calculateWeight(float weightADC)
{
    return ((weightADC - 324) * 3);
}

void Arm::HandleWeighing()
{
      if (weightTimer.checkExpired(true)){
            if (weight_count < 50){
                weightSum += analogRead(LOAD_CELL_PIN);
                weight_count += 1;
                #ifdef __LOAD_CELL_DEBUG__
                plotVariable("weight ADC", analogRead(WEIGHING_PIN));
                plotVariable("weight (g)", calculateWeight(analogRead(WEIGHING_PIN)));
                #endif
                EnterIdle();
                isDoneWeighing = true;
            }
        }
}
