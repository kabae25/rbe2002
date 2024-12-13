/**
 * robot-remote.cpp implements features of class Robot that are related to processing
 * remote control commands. It also manages modes. You might want to trim away some of 
 * the code that isn't needed later in the term.
 */
#include "robot.h"

#include <ir_codes.h>
#include <IRdecoder.h>

/**
 * IRDecoder decoder is declared extern in IRdecoder.h (for ISR purposes), 
 * so we _must_ name it decoder.
 */
#define IR_PIN 17
IRDecoder decoder(IR_PIN);

void Robot::HandleKeyCode(int16_t keyCode)
{ 
    Serial.println(keyCode);

    // Regardless of current state, if ENTER is pressed, go to idle state
    if(keyCode == STOP_MODE) EnterIdle();

    // The SETUP key is used for tuning motor gains
    else if(keyCode == SETUP_BTN)
    {
        if(robotCtrlMode == CTRL_SETUP) {EnterTeleopMode(); EnterIdle();}
        else {EnterSetupMode(); EnterIdle();}
    }

    // If PLAY is pressed, it toggles control mode (setup -> teleop)
    else if(keyCode == PLAY_PAUSE) 
    {
        if(robotCtrlMode == CTRL_AUTO) {EnterTeleopMode(); EnterIdle();}
        else if(robotCtrlMode == CTRL_TELEOP) {EnterAutoMode(); EnterIdle();}
    }

    
    /* 
        Enter Auto Commands Here:
     */
    if(robotCtrlMode == CTRL_AUTO)
    {
        int k = 0 ; // temp variable for the node changes
        switch(keyCode)
        {
            case VOLminus: // vol- sets the desired i node
                k = keyString.toInt();
                Serial.print("Goal i node: "); Serial.println(k);
                igoal = k;
                keyString = "";
                break;
            case VOLplus: // vol+ sets the disired j node
                k = keyString.toInt();
                Serial.print("Goal j Node: "); Serial.println(k);
                jgoal = k;
                keyString = "";
                break;
            case REWIND: // enter the nav state when the rewind button is pressed
                robotState = DRIVING_BIN;
                atPlatform = false;
                // EnterNavIdle();
                //EnterNavLining(baseSpeed);
                // EnterNavTurning(HelperNavCalculateDirection());
                baseSpeed = keyString.toInt();
                keyString = "";
                break;
            case STOP_MODE:
                break;
            case NUM_1:
                arm.raiseArm(false);  
                break;
            case NUM_2:
                arm.lowerArm(false);
                break;
            case NUM_3:
                keyString += (char)(keyCode + 33);
                break;
            case NUM_4:
                chassis.SetTwist(10, 0);
                break;
            case NUM_5:
                chassis.SetTwist(-10, 0);
                break;
            case NUM_6:
                keyString += (char)(keyCode + 32);
                break;
            case NUM_7:
                chassis.Stop();
                break;
            case NUM_8:
            case NUM_9:
                keyString += (char)(keyCode + 31);
                break;
            case NUM_0_10:
                keyString += '0';
                break;
        }
    }

    /**
     * Enter TELEOP commands here
     */
    else if(robotCtrlMode == CTRL_TELEOP)
    {
        switch(keyCode)
        {
            // turning debug
            case UP_ARROW: // set the desired heading to north
                //robotState = ROBOT_NAVIGATING;
                EnterNavTurning(1);
                break;
            case RIGHT_ARROW: // set the desired heading to east
                //robotState = ROBOT_NAVIGATING;
                EnterNavTurning(0);
                break;
            case DOWN_ARROW: // set the desired heading to south
                //robotState = ROBOT_NAVIGATING;
                EnterNavTurning(3);
                break;
            case LEFT_ARROW: // set the desired heading to west
                //robotState = ROBOT_NAVIGATING;
                EnterNavTurning(2);
                break;
            case NUM_1:

                arm.raiseArm(false);
                break;
            case NUM_2:
                arm.lowerArm(false);
                break;
            case NUM_3:
              EnterSearchingBin();
              break;
            case NUM_4:
              drivingToRamp = true;
              EnterDrivingToDump();
              break;
        }
    }

    /**
     * Enter SETUP mode commands here
     */
    /*
    else if(robotCtrlMode == CTRL_SETUP)
    {
        float k = 0;
        switch(keyCode)
        {
            case VOLminus: // vol- sets the KP to the numbers entered divided by 100
                k = keyString.toInt() / 100.0;
                Serial.print("Kp = "); Serial.println(k);
                chassis.SetMotorKp(k);
                keyString = "";
                break;
            case PLAY_PAUSE: // play/pause sets the KI of the motor controllers to 
                k = keyString.toInt() / 100.0;
                Serial.print("Ki = "); Serial.println(k);
                chassis.SetMotorKi(k);
                keyString = "";
                break;
            case VOLplus:
                k = keyString.toInt() / 100.0;
                Serial.print("Kd = "); Serial.println(k);
                chassis.SetMotorKd(k);
                keyString = "";
                break;
            case UP_ARROW:
                if(!keyString.length()) chassis.SetWheelSpeeds(60, 60); // set motor speed to 10 cm/s
                break;
            case DOWN_ARROW:
                if(!keyString.length()) chassis.SetWheelSpeeds(20, 20);
                break;
            case ENTER_SAVE:
                keyString = "";
                chassis.SetWheelSpeeds(0, 0);
                break;
            case NUM_1:
            case NUM_2:
            case NUM_3:
                keyString += (char)(keyCode + 33);
                break;
            case NUM_4:
            case NUM_5:
            case NUM_6:
                keyString += (char)(keyCode + 32);
                break;
            case NUM_7:
            case NUM_8:
            case NUM_9:
                keyString += (char)(keyCode + 31);
                break;
            case NUM_0_10:
                keyString += '0';
                break;
        }
    }
    */
    
}


/*
 * CONTROL MODE CHANGES
 */
void Robot::EnterTeleopMode(void)
{
    controlModeChange("TELEOP");
    robotCtrlMode = CTRL_TELEOP;
}

void Robot::EnterAutoMode(void)
{
    controlModeChange("AUTO");
    robotCtrlMode = CTRL_AUTO;
}

void Robot::EnterSetupMode(void)
{
    controlModeChange("SETUP");
    robotCtrlMode = CTRL_SETUP;
}