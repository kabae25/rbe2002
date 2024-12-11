#pragma once
#include <Arduino.h>

class Vision {
    public:
        struct AprilTagData
        {
            int16_t header,
            checksum, 
            id = 0;
            float w, 
            h, 
            rot, 
            x, 
            y, 
            z, 
            rx, 
            ry, 
            rz; 
            int16_t nul;
        };
        

    // initialize the camera
    void init(void);

    uint8_t FindAprilTags(AprilTagData&);
};
