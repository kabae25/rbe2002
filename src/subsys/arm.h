#pragma once
#include <servo32u4.h>

class Arm {
    public:
        void enterInit();

        void update();

        void raiseArm() {raiseCmd = true;}
        void lowerArm() {lowerCmd = true;}
        void weigh() {weighCmd = true;}
    protected:
        bool checkRaiseCmd() {
            if(raiseCmd) {
                raiseCmd = false;
                return true;
            } else return false;
        }
        bool checkLowerCmd() {
            if(lowerCmd) {
                lowerCmd = false;
                return true;
            } else return false;
        }
        bool checkWeighCmd(){
            if(weighCmd) {
                weighCmd = false;
                return true;
            } else return false;
        }
        
        bool raiseCmd = false;
        bool lowerCmd = false;
        bool weighCmd = false;

        void EnterIdle();
        void HandleIdle();
        void EnterRaising();
        void HandleRaising();
        void EnterLowering();
        void HandleLowering();
        void EnterWeighing();
        void HandleWeighing();
        enum States {INIT, IDLE, RAISING, LOWERING, WEIGHING};
        States state = INIT;
};