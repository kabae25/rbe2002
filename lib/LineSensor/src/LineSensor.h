#pragma once
#include <Arduino.h>

// constants file doesn't work in lib directory
#define LEFT_INNER_SENSOR A6
#define RIGHT_INNER_SENSOR A4
#define LEFT_OUTER_SENSOR A11
#define RIGHT_OUTER_SENSOR A0

#define EXPO 0.9

#define BRIGHT_THRESHOLD 1200
#define EDGE_THRESHOLD 900
#define INTERSECTION_LOWER_THRESHOLD 600
#define INTERSECTION_UPPER_THRESHOLD 900
#define EDGE_THRESHOLD 900

#define LINE_FOLLOWER_KP 0.0055
#define LINE_FOLLOWER_KD 0.0007

#define LINE_FOLLOW_LINEAR_SPEED 20

class LineSensor
{
protected:
    uint8_t outerLeftSensorPin = LEFT_OUTER_SENSOR;
    uint8_t outerRightSensorPin = RIGHT_OUTER_SENSOR;
    
    uint8_t innerLeftSensorPin = LEFT_INNER_SENSOR;
    uint8_t innerRightSensorPin = RIGHT_INNER_SENSOR;

    bool prevOnIntersection = false;

public:
    LineSensor(void) {}
    void Initialize(void);
    void setKp(float newKp);
    void setKi(float newKi);
    void setKd(float newKd);
    void resetController(void);
    int16_t CalcEffort(int16_t error);
    int16_t CalcError(void);
    bool CheckIntersection(void);
    bool checkOffLine(void);
    bool CheckEdge(void);
};