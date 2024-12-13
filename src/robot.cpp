#include <robot.h>
#include <IRdecoder.h>
#include <Esp32.h>
#include <Vision.h>
#include <subsys/arm.h>

void Robot::InitializeRobot(void)
{
    // Hardware
    imu.init();
    decoder.init();
    lineSensor.Initialize();
    esp32.init();
    vision.init();
    imuSubTimer.start(90);

    // Subsystems
    chassis.InititalizeChassis();
    arm.enterInit();
    EnterNavIdle();
}

void Robot::EnterInit()
{
    arm.raiseArm(false);
    robotState = INIT;
}

void Robot::EnterIdle()
{
    robotState = IDLE;
}

void Robot::HandleIdle()
{
    // receive a begin state from mqtt
}

void Robot::EnterDrivingToBin(Tag goal_tag)
{
    igoal = goal_tag.x;
    jgoal = goal_tag.y;
    
    robotState = DRIVING_BIN;
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Driving to Bin State");
    #endif
}

void Robot::HandleLiningToBin()
{
    robotShouldBeMoving = true;
}

void Robot::EnterSearchingBin()
{
    robotState = SEARCHING_BIN;
    arm.lowerArm(false);
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Searching Bin State");
    #endif
}

void Robot::EnterCollectingBin()
{
    robotState = COLLECTING;
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Collecting Bin State");
    #endif
}

void Robot::HandleSearchingBin() {
  chassis.SetTwist(0, 0.9);
  if(vision.FindAprilTags(tag)) {
    chassis.Stop();
    EnterCollectingBin();
  }
}

void Robot::HandleCollectingBin()
{
    if (vision.FindAprilTags(tag)) { // calculate the trust of the april tag reading
        if (-tag.z < 3.5) {
          Serial.println("too close");
          EnterWeighingBin();
          // arm.raiseArm(true);
        }
        else { // go to the april tiag
        Serial.println("Moving to apirltag");
            float rot_error = tag.x; // center - x
            float rot_Kp = 0.25;
            float rot_effort = (rot_Kp * rot_error);

            float forward_error = 3 - (-tag.z);
            float forward_Kp = 3;
            float forward_effort = (forward_Kp * forward_error);
            chassis.SetTwist(forward_effort, rot_effort);
        }
    }
}

void Robot::EnterWeighingBin()
{
    Serial.println("Entering Weighing Bin State");
    alignTimerStartTime = millis() / 1000;
    robotState = WEIGHING;
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Weighing State");
    #endif
}

void Robot::HandleWeighingBin()
{ 
    Serial.println((millis() / 1000) - alignTimerStartTime);
    if ((millis() / 1000) - alignTimerStartTime > 3) {
        chassis.SetWheelSpeeds(0, 0);
        arm.raiseArm(true);
        if(arm.checkWeighingComplete()) {
            EnterDrivingToRamp();
        }
    } else {
      chassis.SetWheelSpeeds(-2.7, -2.7);
    }
}

void Robot::EnterDrivingToRamp()
{
    robotState = DRIVING_RAMP;
    retreatingFromBin = true;
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Driving to Ramp State");
    #endif
}

void Robot::HandleDrivingRamp()
{
    if (retreatingFromBin){
        if (startingEncoderForMove > chassis.getEncoder()){
                chassis.SetTwist(5, 0);
            }
            else {
                retreatingFromBin = false;
                turnToRamp = true;
                chassis.SetTwist(0, 0.3);
            }
    }
    else {
        if (turnToRamp){
            if (currDirection == 1){
                Serial.println("Done turning to ramp");
                drivingToRamp = true;
                EnterDrivingToDump();
            }
        }
    }
}



void Robot::EnterDrivingToDump()
{
    robotState = DRIVING_DUMP;
    EnterNavLining();
    #ifdef __STATE_DEBUG_
        Serial.println("Entering Driving up Ramp to Dump State");
    #endif
}

void Robot::HandleDrivingDump()
{
    if (ramping && !prevRamping){
        prevRamping = true;
    }
    else if (!ramping && prevRamping){
        prevRamping = false;
        EnterNavIdle();
        EnterDumpingBin();
    }
}

void Robot::EnterDumpingBin()
{
    
    robotState = DUMPING;
    chassis.SetTwist(5, 0);
    #ifdef __STATE_DEBUG_
        Serial.println("Entering dumping bin State");
    #endif
}

void Robot::HandleDumpingBin()
{
    if (lineSensor.CheckEdge() && !gottenToEdge) {
        startingEncoderForMove = chassis.getEncoder();
        gottenToEdge = true;
        chassis.SetTwist(-2, 0);
    }
    if (gottenToEdge && ((startingEncoderForMove - 5 * chassis.LEFT_CM_S_TO_TICKS_INT) > chassis.getEncoder())) {
        spinning180 = true;
        chassis.SetTwist(0, 0.5);
    }
    if (spinning180 && (currDirection == 3)) {
      Serial.println("Done spinning");
        arm.EnterLowering();
        chassis.SetTwist(0,0);
        EnterIdle();
    }
    // once arm is lowered
    // go to returning home state
}

void Robot::EnterReturningHome()
{
    robotState = RETURNING;
    #ifdef __STATE_DEBUG_
        Serial.println("Entering returning to start position State");
    #endif
}

void Robot::HandleReturningHome()
{
    EnterIdle();
}

void Robot::HandleOrientationUpdate(void)
{
    if (!stabilized) {
        float biasX = 0.96 * imu.gyroBias.x + (1-0.96) * imu.g.x;
        float biasY = 0.95 * imu.gyroBias.y + (1-0.95) * imu.g.y;
        float biasZ = 0.98 * imu.gyroBias.z + (1-0.98) * imu.g.z;

        if (imuSubTimer.checkExpired(true)) {
            biasDelta = 0.98*biasDelta + (1-0.98)*((imu.gyroBias.z - prevZBias) / 0.09); // get imu drift per second
            //plotVariable("Bias Filter", abs(biasDelta));
            prevZBias = imu.gyroBias.z;

            if (abs(biasDelta) < 0.5) { // was 0.25 // once imu delta has settled,
                imuSubTimer.cancel();
                stabilized = true;
            }
        }

        imu.gyroBias.x = biasX;
        imu.gyroBias.y = biasY;
        imu.gyroBias.z = biasZ;

        Serial.println("Not Ready");
    }
    else {
        // once the bias is tolerable, enable the complimentary filter and send the imu angles
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
        //plot("", imu.a.z);
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
    else if ((eulerAngles.y > -3.0) && (ramping == true)) {
        atPlatform = true;
        ramping = false;
        digitalWrite(13, LOW);
    }
}

void Robot::EnterNavTurning(int cardinal) {
    atTheGoalGridNode = true;
    robotShouldBeMoving = true;
    goalDirection = cardinal; // cardinal is the desired direction
    numTurns = goalDirection - currDirection;
    
    navigatingState = NAVIGATING_TURNING;
    Serial.println("Entering NAVIGATING: TURNING");
}

// Nav: LINING state transition
void Robot::EnterNavLining() {
    robotShouldBeMoving = true;
    navigatingState = NAVIGATING_LINING;
    Serial.println("Entering NAVIGATING: LINING");
}

// Nav: PULLUP state transition
void Robot::EnterNavPullup(int time) {
    robotShouldBeMoving = true;
    targetTime = time;

    navigatingState = NAVIGATING_PULLUP;
    Serial.println("Entering NAVIGATING: PULLUP");
}

// Nav: IDLE state transition
void Robot::EnterNavIdle(void) {
    chassis.Stop();
    robotShouldBeMoving = false;    
    
    navigatingState = NAVIGATING_IDLE;
    Serial.println("Entering NAVIGATING: IDLE");
}

// This state can decide what direction to turn based on some destination and current coordinates on the manhatten grid
void Robot::HandleNavTurning(void) {
    if(!looking) {
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
                if(edgeDetected) {
                    arm.lowerArm(true);
                    if (arm.checkArmRaised()) {
                        /** Go back down ramp and some arbitrary grid location */
                        jgoal = 1;
                        igoal = 1;
                        EnterNavLining();
                    }
                } else {
                    EnterNavLining();
                }
            }   
        else { // reached destination
            atTheGoalGridNode = true;
            EnterSearchingBin();
        }
   }
}

// Nav: handle LINING state
void Robot::HandleNavLining(void) {
    if ((!lineSensor.CheckIntersection()) || drivingToRamp) { // || ((prevRamping == false) && (ramping == true))

        float lineError = lineSensor.CalcError();
        float lineErrorDerivative = lineError - prevLineError;
        float turnEffort = LINE_FOLLOWER_KP * lineError + LINE_FOLLOWER_KD * lineErrorDerivative;
        
        chassis.SetTwist(LINE_FOLLOW_LINEAR_SPEED, -turnEffort);
        prevLineError = lineError;   
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
        // EnterNavTurning(HelperNavCalculateDirection(J));
    }
}

// Nav: handle IDLE state
void Robot::HandleNavIdle(void) {
    
}

// Calculate what heading to face in order to get to the goal destination
int Robot::HelperNavCalculateDirection(ORDER direction_to_go_first) {
    int retVal = 0;

    // then calculate the direction necessary to face
    switch (direction_to_go_first) {
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

void Robot::RobotLoop(void) 
{
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    if(chassis.CheckChassisTimer())
    {
        chassis.UpdateMotors(); // make da motors move

        if (robotShouldBeMoving) { //ONLY USE WHEN IN A LINE FOLLOWING STATE
            switch (navigatingState) { 
                case (NAVIGATING_TURNING): 
                    HandleNavTurning(); break;
                case (NAVIGATING_LINING): 
                    HandleNavLining(); break;
                case (NAVIGATING_PULLUP) :
                    HandleNavPullup(); break;
            }
        }
    }

    switch (robotState) {
        case (IDLE):
            HandleIdle(); break;
        case DRIVING_BIN:
            HandleLiningToBin(); break;
        case SEARCHING_BIN:
            HandleSearchingBin(); break;
        case COLLECTING: 
            HandleCollectingBin(); break;
        case WEIGHING:
            HandleWeighingBin(); break;
        case DRIVING_RAMP:
            HandleDrivingRamp(); break;
        case DRIVING_DUMP:
            HandleDrivingDump(); break;
        case DUMPING:
            HandleDumpingBin(); break;
        case RETURNING:
            HandleReturningHome(); break;
    }
    
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
    }

    // misc
    lineSensor.CalcError();
    esp32.listenUART();
    arm.update();
}