#include "LineSensor.h"


// initialize the linesensors
void LineSensor::Initialize(void)
{
    pinMode(LEFT_INNER_SENSOR, INPUT);
    pinMode(LEFT_OUTER_SENSOR, INPUT);
    pinMode(RIGHT_INNER_SENSOR, INPUT);
    pinMode(RIGHT_OUTER_SENSOR, INPUT);

}

int16_t LineSensor::CalcError(void) 
{

    // Scaling on sensors. 1 is on the instide, 3 the outside

    int16_t color = 1023; // 1023 corresponds to white, 0 to black

    // Calculate avg left and right weighted errors by taking each pin and multipling by some weight
    // int16_t left_error = ((color - analogRead(leftSensorPin1))*weight1 + (color - analogRead(leftSensorPin2))*weight2 + (color - analogRead(leftSensorPin3))*weight3);
    // int16_t right_error = ((color - analogRead(rightSensorPin1))*weight1 + (color - analogRead(rightSensorPin2))*weight2 + (color - analogRead(rightSensorPin3))*weight3);
    
    // Calculate the raw line error
    int16_t error = analogRead(LEFT_OUTER_SENSOR) - analogRead(RIGHT_INNER_SENSOR);
    //Serial.print(analogRead(leftSensorPin));
    //Serial.print(" ");
    //Serial.println(analogRead(RIGHT_OUTER_SENSOR));

    return error;
}

float Kp = 0.005; // 0.0015 // 0.005 // 0.0025 for 20 cm/s 0.0018 // really bad: 0.003
float Ki = 0.0;
float Kd = 0.0005; // 0.004

// change PID term gains on the fly
void LineSensor::setKp(float newKp) {Kp = newKp;}
void LineSensor::setKi(float newKi) {Ki = newKi;}
void LineSensor::setKd(float newKd) {Kd = newKd;}

// error variables for PID controller
int16_t sumError = 0;
int16_t prevError = 0;

// reset the sumError and prevError accumulators
void LineSensor::resetController(void) {
    sumError = 0;
    prevError = 0;
}

// Calculate the effort returned by the PID controller
int16_t LineSensor::CalcEffort(int16_t error) {
    sumError+=error; // accumulate integral error

    int16_t d_error = error - prevError; // get the derivative error
    prevError = error;
    return (Kp * error + Ki * sumError + Kd * d_error); // calculate PID effort
}

// check to see if there is an intersection
bool LineSensor::CheckIntersection(void)
{
    bool retVal = false;

    //bool isLeftLowerDark = analogRead(A11) > INTERSECTION_LOWER_THRESHOLD;
    //bool isLeftUpperDark = analogRead(A11) < INTERSECTION_UPPER_THRESHOLD;
    //bool isRightLowerDark = analogRead(A0) > INTERSECTION_LOWER_THRESHOLD;
    //bool isRightUpperDark = analogRead(A0) < INTERSECTION_UPPER_THRESHOLD;

    // // Calculate avg left and right weighted errors by taking each pin and multipling by some weight
    // int16_t left_error = ((color - analogRead(leftSensorPin1))*weight1 + (color - analogRead(leftSensorPin2))*weight2 + (color - analogRead(leftSensorPin3))*weight3);
    // int16_t right_error = ((color - analogRead(rightSensorPin1))*weight1 + (color - analogRead(rightSensorPin2))*weight2 + (color - analogRead(rightSensorPin3))*weight3);

    bool isLeftBright = analogRead(LEFT_OUTER_SENSOR) > INTERSECTION_LOWER_THRESHOLD;
    bool isRightBright = analogRead(RIGHT_OUTER_SENSOR) > INTERSECTION_UPPER_THRESHOLD;

    bool onIntersection = isLeftBright && isRightBright;
    if(onIntersection && !prevOnIntersection) retVal = true;

    prevOnIntersection = onIntersection;

    return retVal;
}

bool LineSensor::CheckEdge(void)
{
    bool retVal = false;
    bool leftEdge = analogRead(LEFT_OUTER_SENSOR) > EDGE_THRESHOLD;
    bool rightEdge = analogRead(RIGHT_OUTER_SENSOR) > EDGE_THRESHOLD;
    if (leftEdge && rightEdge)
    {
        retVal = true;
    }
    return retVal;
}