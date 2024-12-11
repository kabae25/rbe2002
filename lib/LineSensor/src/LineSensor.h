#pragma once

#include <Arduino.h>

// #define LEFT_LINE_SENSOR3 A4
#define LEFT_LINE_SENSOR A0
// #define LEFT_LINE_SENSOR1 A11

#define RIGHT_LINE_SENSOR A4
// #define RIGHT_LINE_SENSOR2 A3
// #define RIGHT_LINE_SENSOR3 A2

#define EXPO 0.9

class LineSensor
{
protected:
    uint8_t leftSensorPin = LEFT_LINE_SENSOR;
    uint8_t rightSensorPin = RIGHT_LINE_SENSOR;

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