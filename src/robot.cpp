#include <robot.h>
#include <IRdecoder.h>
#include <Esp32.h>
#include <Vision.h>
#include <arm.h>

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();

    /**
     * Initialize the IMU and set the rate and scale to reasonable values.
     */
    imu.init();


    pinMode(13, OUTPUT);

    lineSensor.Initialize();

    esp32.init();

    // initialize the camera
    vision.init();

    /* servo init */
    arm.init();


    //loadCellHX1.Init();
}

void Robot::EnterIdleState(void)
{
    chassis.Stop(); 
}

/**
 * Here is a good example of handling information differently, depending on the state.
 * If the Romi is not moving, we can update the bias (but be careful when you first start up!).
 * When it's moving, then we update the heading.
 */
void Robot::HandleOrientationUpdate(void)
{
    if(robotCtrlMode == CTRL_SETUP)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
        //imu.updateAcceBias();
    }

    else // update orientation
    {
        /*
        * Using moving average to find robot eulerAngles with the gyroscope (prone to drift)
        */
        eulerAngles.x += (imu.mdpsPerLSB*(imu.g.x - imu.gyroBias.x)*(1/imu.gyroODR))/1000;
        eulerAngles.z += (imu.mdpsPerLSB*(imu.g.z - imu.gyroBias.z)*(1/imu.gyroODR))/1000;


        /*
         * Pitch axis complementary filter 
         */
        double prediction = eulerAngles.y + ((1/imu.gyroODR)*(imu.mdpsPerLSB/1000)*(imu.g.y - imu.gyroBias.y));

        double observation = (180/PI) * atan2(-imu.a.x, imu.a.z);

        double KAPPA = 0.01; // 0.009 worked well // 0 relies purely on the prediction from the gyro
        eulerAngles.y = (1-KAPPA)*prediction + KAPPA*(observation);

        double EPSILON = 0.01; // Factor of new gyro bias adjustment

        if (KAPPA != 0) {
            imu.gyroBias.y -= EPSILON*(1.0/((imu.mdpsPerLSB/1000.0) * imu.gyroODR))*(observation - prediction);
        }

        // Update other variables relating to the heading
        utilUpdatePitchIndication();
        utilUpdateCurrHeading();

        #ifdef __NAV_DEBUG__
        plot("currDirection", currDirection); // plot the imu data
        #endif

    }

    #ifdef __IMU_DEBUG__
        //plot("Time", millis());
        plot("Roll", eulerAngles.x);
        plot("Pitch", eulerAngles.y);
        plot("Yaw", eulerAngles.z);
        //plot("Bias X", imu.gyroBias.x);
        plot("Bias Y", imu.gyroBias.y);
        //plot("Bias Z", imu.gyroBias.z);
        //plot("Acc X", imu.a.x);
        //plot("Acc Y", imu.a.y);
        //plot("Acc Z", imu.a.z);
    #endif
}

void Robot::utilUpdateCurrHeading(void) {
    if((int(eulerAngles.z) % 360 > -5) && (int(eulerAngles.z) % 360 < 5)){
            currDirection = 0;
        } else if (((int(eulerAngles.z) % 360 > 85) && (int(eulerAngles.z) % 360 < 95)) || ((int(eulerAngles.z) % 360 < -265) && (int(eulerAngles.z) % 360 > -275))){
            currDirection = 1;
        } else if (((int(eulerAngles.z) % 360 > 175) && (int(eulerAngles.z) % 360 < 185)) || ((int(eulerAngles.z) % 360 < -175) && (int(eulerAngles.z) % 360 > -185))){
            currDirection = 2;
        } else if (((int(eulerAngles.z) % 360 > 265) && (int(eulerAngles.z) % 360 < 275)) || ((int(eulerAngles.z) % 360 < -85) && (int(eulerAngles.z) % 360 > -95))){
            currDirection = 3;
        }
}

void Robot::utilUpdatePitchIndication(void) {
    if ((eulerAngles.y < -10.0) && (ramping == false)) {
        ramping = true;
        digitalWrite(13, HIGH);
    }
    else if ((eulerAngles.y > -10.0) && (ramping == true)) {
        atPlatform = true;
        ramping = false;
        digitalWrite(13, LOW);
    }

    /*
    if ((eulerAngles.y > 10.0) && (ramping == false)) {
        ramping = true;
        digitalWrite(13, HIGH);
    }
    else if ((eulerAngles.y < 10.0) && (ramping == true)) {
        ramping = false;
        atPlatform = false;
        digitalWrite(13, LOW);
    }
    */
}

/**
 * TITLE: Methods relating to the navigating state machine
 */
// Nav: TURNING state transition
void Robot::EnterNavTurning(int cardinal) {
    goalDirection = cardinal; // cardinal is the desired direction
    numTurns = goalDirection - currDirection;
    navigatingState = NAVIGATING_TURNING;
    Serial.println("Entering NAVIGATING: TURNING");
}

// Nav: LINING state transition
void Robot::EnterNavLining(int speed) {
    baseSpeed = speed;

    navigatingState = NAVIGATING_LINING;
    Serial.println("Entering NAVIGATING: LINING");
}

// Nav: PULLUP state transition
void Robot::EnterNavPullup(int time) {
    targetTime = time;
    navigatingState = NAVIGATING_PULLUP;
    Serial.println("Entering NAVIGATING: PULLUP");
}

// Nav: IDLE state transition
void Robot::EnterNavIdle(void) {
    chassis.Stop();
    navigatingState = NAVIGATING_IDLE;
    Serial.println("Entering NAVIGATING: IDLE");
}

// Nav: handle TURNING state
void Robot::HandleNavTurning(void) {
    if (currDirection != goalDirection) {
        if (numTurns > 0) {
            if (numTurns > 2) {chassis.SetTwist(0, -2);} 
            else {chassis.SetTwist(0, 2);}
        }
        else {
            if (numTurns < -2) {chassis.SetTwist(0, 2);} 
            else {chassis.SetTwist(0, -2);}
        }
    }
    else if (igoal != icurr || jgoal != jcurr) {
        EnterNavLining(baseSpeed);
    }
    else { // reached destination
        navLeg++;
        EnterNavIdle();
    }
}

// Nav: handle LINING state
void Robot::HandleNavLining(void) {
    if ((!lineSensor.CheckIntersection())) { // || ((prevRamping == false) && (ramping == true))
        // line following is handled sychronously in the robotLoop method
    }
    else if (atPlatform) {
        currTime = millis();
        EnterNavPullup(millis() + 1000); // pull up to the center(ish) of the intersection
    }
    else {
        currTime = millis();
        EnterNavPullup(millis() + 1000); // pull up to the center(ish) of the intersection
    }
}

// Nav: handle PULLUP state
void Robot::HandleNavPullup(void) {
    if(currTime < targetTime){
        chassis.SetWheelSpeeds(8,8);
        currTime = millis();
    } else {
        switch (currDirection) {
            case (0):
                icurr++;
                break;
            case (1):
                jcurr++;
                break;
            case (2):
                icurr--;
                break;
            case (3):
                jcurr--;
                break;
        }
        targetTime = 0;
        chassis.Stop();
        EnterNavTurning(HelperNavCalculateDirection());
    }
}

// Nav: handle IDLE state
void Robot::HandleNavIdle(void) {
    // For just going up the ramp and stopping
    switch(navLeg) {
        case 0: // set the goal i and j, then start navigating to the node
            igoal = 0;
            jgoal = 1;
            EnterNavTurning(HelperNavCalculateDirection());
            break;
        case 1:
            igoal = 1;
            jgoal = 1;
            EnterNavTurning(HelperNavCalculateDirection());
            break;
        case 2:
            // completed the nav so do nothing...
            // otherwise probably make this change the robot state
            break;
    }
    /*if (andBack) { // swap variables if the andBack flag was raised
        int tempi = igoal;
        int tempj = jgoal;
        igoal = istarting;
        jgoal = jstarting;
        istarting = tempi;
        jstarting = tempj;
        EnterNavTurning(HelperNavCalculateDirection());
    }
    */
    // possible do someting here...
}

// Calculate what heading to face in order to get to the goal destination
int Robot::HelperNavCalculateDirection(void) {
    int retVal = 0;

    // first see if a direction change is necessary
    // then decide what order to check i or j
    enum ORDER {I, J,};
    ORDER order = I;
    
    if (!andBack) { // normally do i then j
        if (icurr != igoal) {
            order = I;
        }
        else if (jcurr != jgoal) {
            order = J;
        }
    }
    else { // on the way back, do j then i
        if (jcurr != jgoal) {
            order = J;
        }
        if (icurr != igoal) {
            order = I;
        }
    }

    // then calculate the direction necessary to face
    switch (order) {
        case (I):
            if ((igoal - icurr) > 0) retVal = 0; // if positive go east
            else if ((igoal - icurr) < 0)  retVal = 2; // if negative go west
            else retVal = currDirection;
            break;
        case (J):
            if ((jgoal - jcurr) > 0) retVal = 1; // if positive go north
            else if ((jgoal - jcurr) < 0) retVal = 3; // if negative go south
            else retVal = currDirection;
            break;
    }
    return retVal;
}

void Robot::HelperLineFollowingUpdate(void) {
    int16_t effort = lineSensor.CalcEffort(lineSensor.CalcError());
    chassis.SetTwist(baseSpeed, effort);
}

void Robot::EnterManIdle(void) {
    chassis.Stop();
    manipulatingState = MANIPULATING_IDLE;
    Serial.println("Entering MANIPULATING: Idle");
}

void Robot::EnterManSearching(void) {
    chassis.Stop();
    chassis.SetTwist(0, 1);
    manipulatingState = MANIPULATING_SEARCHING;
    Serial.println("Entering MANIPULATING: Searching");

}

void Robot::EnterManApproaching(void) {
    chassis.Stop();
    arm.lower();
    timestamp = (millis() + remembrance);
    manipulatingState = MANIPULATING_APPROACHING;
    Serial.println("Entering MANIPULATING: Approaching");

}

void Robot::EnterManLifting(void) {
    arm.raise();
    manipulatingState = MANIPULATING_LIFTING;
    Serial.println("Entering MANIPULATING: Lifting");

}

void Robot::EnterManWeighing(void) {
    weightTimer.start(20);
    manipulatingState = MANIPULATING_WEIGHING;
    Serial.println("Entering MANIPULATING: Weighing");

}

void Robot::HandleManIdle(void) {
    
}

void Robot::HandleManSearching(void) {
    if (vision.FindAprilTags(tag)) {
        EnterManApproaching();
    }
    chassis.SetTwist(0, 0.5);

}


void Robot::HandleManApproaching(void) {

    // calculate the trust of the april tag reading
    if (vision.FindAprilTags(tag)) {
        trust = 1;
        timestamp = (millis() + remembrance);
    }
    else {
        trust = (timestamp - millis()) / remembrance;
        if (trust <= 0) {
            EnterManSearching();
        }
    }

    plot("Trust", trust);
    plot("tag x", tag.x);
    plot("tag.z", -tag.z);

    float rot_error = 0 - (tag.x); // center - x
    float rot_Kp = 0.5;
    float rot_effort = trust * (rot_Kp * rot_error);

    float forward_error = 3 - (-tag.z);
    float forward_Kp = 5;
    float forward_effort = trust * (forward_Kp * forward_error);

    chassis.SetTwist(forward_effort, -rot_effort);

    if (HelperCheckApproachComplete(3.5, -tag.z)) {
        EnterManLifting();
    }
}

void Robot::HandleManLifting(void) {
    if (arm.raise()) {
        EnterManWeighing();
    }
}

void Robot::HandleManWeighing(void) {
    if (weightTimer.checkExpired(true)) {
        unsigned int sample = digitalRead(LOAD_CELL_PIN);
        mass = mass*0.8 + (1-0.8)*((sample - 310.86)/1.2548);
        
        if (weight_count > 50) {
            Serial.print("Mass: ");
            Serial.print(mass);
            weight_count = 0;
            EnterManIdle();
        }
        else {
            weight_count++;
        }
    }
}

uint8_t Robot::HelperCheckApproachComplete(int headingTolerance, int distanceTolerance) {

    if (distanceTolerance <= headingTolerance) {
        return 1;
    }
    else return 0;
}




void Robot::RobotLoop(void) 
{
    /**
     * The main loop for your robot. Process both synchronous events (motor control),
     * and asynchronous events (IR presses, distance readings, etc.).
    */

    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Check the Chassis timer, which is used for executing motor control
     */
    if(chassis.CheckChassisTimer())
    {
        // add synchronous, pre-motor-update actions here
        if(navigatingState == NAVIGATING_LINING)
        {
            HelperLineFollowingUpdate();
        }

        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }

    /**
     * TITLE: Robot State handler calls
     */
    switch (robotState) {
        case (ROBOT_IDLE):
            // do nothing for now, maybe some time do something more important...
            break;
        case (ROBOT_NAVIGATING):
            switch (navigatingState) { // handle the robot's navigating state machine
                case (NAVIGATING_IDLE): 
                    HandleNavIdle();
                    break;
                
                case (NAVIGATING_TURNING): 
                    HandleNavTurning();
                    break;
                
                case (NAVIGATING_LINING): 
                    HandleNavLining();
                    break;
                
                case (NAVIGATING_PULLUP) :
                    HandleNavPullup();
                    break;
                
            }
            break;
        case (ROBOT_MANIPULATING):
            switch (manipulatingState) {
                case (MANIPULATING_IDLE):
                    HandleManIdle();
                    break;
                case (MANIPULATING_SEARCHING):
                    HandleManSearching();
                    break;
                case (MANIPULATING_APPROACHING):
                    HandleManApproaching();
                    break;
                case (MANIPULATING_LIFTING):
                    HandleManLifting();
                    break;
                case (MANIPULATING_WEIGHING):
                    HandleManWeighing();
                    break;
            }
            // some stuff will go here in the future;
            break;
    }

    /**
     * Check for an IMU update
     */

    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
    }
    lineSensor.CalcError();

    esp32.listenUART();

    arm.update();

    //int32_t reading = 0;
    //if (loadCellHX1.GetReading(reading)) {
    //    Serial.println(reading);
    //}

}
