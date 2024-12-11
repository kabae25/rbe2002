#pragma once
#include <servo32u4.h>

class Arm {
    public:
        void init();
        void update();
        bool raise();
        bool lower();
        void weigh();
        void utilSetLifter(uint16_t);
        int measureWeight();
};