#pragma once
/**
 * Put all constant numbers... defines, pins in here
 */

/**
 * Line Sensor
 */
#define LEFT_INNER_SENSOR A6
#define RIGHT_INNER_SENSOR A4
#define LEFT_OUTER_SENSOR A11
#define RIGHT_OUTER_SENSOR A0

#define EXPO 0.9

#define BRIGHT_THRESHOLD 1200;
#define EDGE_THRESHOLD 900;
#define INTERSECTION_LOWER_THRESHOLD 600;
#define INTERSECTION_UPPER_THRESHOLD 900;
#define EDGE_THRESHOLD 1000;

/**
 * Arm
 */
#define AMP_PIN 12 // for the AD620
#define RAISED_PWM 570
#define LOWERED_PWM 1700
#define LOAD_CELL_PIN 10 //TODO CHANGE TO ACTUAL ANALOG PIN