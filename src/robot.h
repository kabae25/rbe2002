#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>
#include <Esp32.h>
#include <Vision.h>
#include "subsys/arm.h"
#include <ZSC31014.h>

#define LOAD_CELL_PIN 10
class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE {
        CTRL_TELEOP,
        CTRL_AUTO,
        CTRL_SETUP,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_SETUP;

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */

    /** 
     * This statemachine controls the actions of the robot while it is in the NAVIGATING state. 
     * This includes going to a target goal i,j; turning automatically, and line following  
    */
    enum NAVIGATING_STATE {
        NAVIGATING_IDLE,
        NAVIGATING_TURNING,
        NAVIGATING_LINING,
        NAVIGATING_PULLUP,
    };

    /**
     * This statemachine controls the actions of the robot while it is in the MANIPULATING state
     * This includes locating and driving to april tags, lifting and weighing trash
     */
    enum MANIPULATING_STATE {
        MANIPULATING_IDLE,
        MANIPULATING_SEARCHING,
        MANIPULATING_APPROACHING,
        MANIPULATING_LIFTING,
        MANIPULATING_WEIGHING
    };



    // set default navigating state
    NAVIGATING_STATE navigatingState = NAVIGATING_IDLE;

    // set default manipulating state
    MANIPULATING_STATE manipulatingState = MANIPULATING_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    /* Line sensor */
    LineSensor lineSensor;

    /* esp32 board */
    Esp32 esp32;

    Vision vision;

    // For managing key presses
    String keyString;

    /* ---- Initialize the Arm ---- */
    Arm arm;

    //HX711 loadCellHX1(2, 3);
    /**
     * The LSM6 IMU that is included on the Romi. We keep track of the orientation through
     * Euler angles (roll, pitch, yaw).
     */
    LSM6 imu;
    LSM6::vector<float> eulerAngles;
    LSM6::vector<float> gyroBias;

    /* baseSpeed is used to drive at a given speed while, say, line following.*/
    float baseSpeed = 0;

    float numTurns = 0; // Number of 90 degree turns to perform
    bool andBack = false; // Flag to keep track of whether or not to return to the original starting i,j
    int currTime = 0; // used in the NavPullUp class
    int targetTime = 0; // used in the NavPullUp class

    int igoal = 0;
    int jgoal = 0;

    int icurr = 0;
    int jcurr = 0;

    int istarting = 0;
    int jstarting = 0;

    int8_t currDirection = 0; // 0 is east, 1 is north, 2 is west, 3 is south
    int8_t goalDirection = 0;

    bool ramping = false;
    bool prevRamping = false;
    int navLeg = 0;
    bool atPlatform = false;

    /* April tag data storage*/
    Vision::AprilTagData tag;

    float trust = 0;
    float timestamp = 0;
    float remembrance = 1000;

    EventTimer weightTimer;
    unsigned int weight_count = 0;
    unsigned int mass = 0;

    bool edgeDetected = false;


public:
    Robot(void) {keyString.reserve(8);} //reserve some memory to avoid reallocation
    void InitializeRobot(void);
    void RobotLoop(void);
    void HandleRollUp(int targetTime);

    void plot(String name, double var) {
        Serial.print(">");
        Serial.print(name);
        Serial.print(": ");
        Serial.println(var);
    }

    void robotStateChange(String newState) {
        Serial.print("Changing Robot State to: ");
        Serial.println(newState);
    }

    void controlModeChange(String newMode) {
        Serial.print("Changing Control Mode to: ");
        Serial.println(newMode);
    }
private:
        enum ROBOT_STATE {
            INIT, // setup stuff
            IDLE, // wait for a goal bin from MQTT
            DRIVING_BIN, // Drive to the bin (using map)
            COLLECTING, // approach and collect the bin
            WEIGHING, // Weigh the bin
            DRIVING_RAMP, // drive to the ramp
            DRIVING_DUMP, // drive up the ramp
            DUMPING, // dump the bin
            RETURNING // return to the start position
        };
        ROBOT_STATE robotState = IDLE;
public:
    void EnterInit();
    void EnterIdle();
    void EnterDrivingBin();
    void EnterCollecting();
    void EnterWeighing();
    void EnterDrivingRamp();
    void EnterDrivingDump();
    void EnterDumping();
    void EnterReturning();

private:
    void HandleIdle();
    void HandleDrivingBin();
    void HandleCollecting();
    void HandleWeighing();
    void HandleDrivingRamp();
    void HandleDrivingDump();
    void HandleDumping();
    void HandleReturning();

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);

    /* Navigating State machine Methods*/
    void EnterNavTurning(int cardinal);
    void EnterNavLining(int speed);
    void EnterNavPullup(int time);
    void EnterNavIdle(void);

    void HandleNavTurning(void);
    void HandleNavLining(void);
    void HandleNavPullup(void);
    void HandleNavIdle(void);

    /* Navigating State Helper Methods */
    int HelperNavCalculateDirection(void);
    void HelperLineFollowingUpdate(void);

    /* Manipulating state machine methods */
    void EnterManIdle(void);
    void EnterManSearching(void);
    void EnterManApproaching(void);
    void EnterManLifting(void);
    void EnterManWeighing(void);

    void HandleManIdle(void);
    void HandleManSearching(void);
    void HandleManApproaching(void);
    void HandleManLifting(void);
    void HandleManWeighing(void);

    /* Manipulating state helper methods */
    uint8_t HelperCheckApproachComplete(int headingTolerance, int distanceTolerance);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);
    void EnterSetupMode(void);


    /**
     * For IMU
     */
    void HandleOrientationUpdate(void);
    void utilUpdateCurrHeading(void);
    void utilUpdatePitchIndication(void);

    bool stabilized = false;
    float biasDelta = 0;
    EventTimer imuSubTimer;
    float prevZBias =0 ;
    

    /* For commanding the lifter servo */
    void SetLifter(uint16_t position);
};
 