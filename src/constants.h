#pragma once


/**
 * Arm
 */
#define AMP_PIN 12 // for the AD620
#define RAISED_PWM 570
#define LOWERED_PWM 1800
#define LOAD_CELL_PIN 10 //TODO CHANGE TO ACTUAL ANALOG PIN

/**
 * Tag/Bin information
 */
typedef struct Tag {
    uint8_t 
    id,
    x,
    y,
    angle; // follows 0 = east, 3 = south system
};

// Tag tag_0 {0, 2, 2, 0};
// Tag tag_1 {0, 2, 3, 0};