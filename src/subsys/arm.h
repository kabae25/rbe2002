#pragma once
#include <servo32u4.h>
#include <event_timer.h>
class Arm {
    public:
        void enterInit();

        void update();

        bool checkArmRaised();
        bool checkArmLowered();
        bool checkWeighingComplete();

        void set(int pos);
        
        void raiseArm(bool needsToBeChecked) {raiseCmd = true; raiseNeedsToBeChecked = needsToBeChecked; }
        void lowerArm(bool needsToBeChecked) {lowerCmd = true; lowerNeedsToBeChecked = needsToBeChecked; }
        void weigh() {weighCmd = true;}
        float calculateWeight(float weightADC);

        bool raiseNeedsToBeChecked = false;
        bool lowerNeedsToBeChecked = false;

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

        EventTimer weightTimer;
        unsigned int weight_count = 0;
        unsigned int mass = 0;
        bool isDoneWeighing = false;
        float weightSum = 0;

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