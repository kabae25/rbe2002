#pragma once

#include <Arduino.h>
#include "event_timer.h"

class Chassis
{
public:
    /**
     * Kinematic parameters default to the spec sheet from Pololu. You'll need to fine
     * tune them.
     */
    const float ROBOT_RADIUS = 14.7 / 2; // robot wheel base radius

    //          CM/S 
    const float LEFT_CM_S_TO_TICKS_INT = (1.0/(3.1416 * 7.0)) * ((12.0 * (3952.0/33.0))) * (20.0/1000.0);
    const float RIGHT_CM_S_TO_TICKS_INT = (1.0/(3.1416 * 7.0)) * ((12.0 * (3952.0/33.0))) * (20.0/1000.0);

    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static uint8_t loopFlag;

    Chassis(void) {}
    void InititalizeChassis(void)
    {
        InitializeMotorControlTimer();
        InitializeMotors();
    }

    void SetMotorKp(float kp);
    void SetMotorKi(float ki);
    void SetMotorKd(float kd);

    bool CheckChassisTimer(void);

    static void Timer4OverflowISRHandler(void);

    void Stop(void) {SetMotorEfforts(0, 0);}
    void SetTwist(float fwdSpeed, float rotSpeed);
    void SetWheelSpeeds(float, float);
    void UpdateMotors(void);

    /**
     * Initialization and Setup routines
     */
    void InitializeMotorControlTimer(void);
    void InitializeMotors(void);
    
    void SetMotorEfforts(int16_t, int16_t);
    long getEncoder();
};
