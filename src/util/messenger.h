#pragma once
#include <Arduino.h>

class Messenger {
    public:
        bool checkIfBegin();
        
        void setBinPtrs(int16_t* binId);

        void init();
        void update(); // parse incoming and outgoing messages
    
    protected:
        void handleBinID(); // parse message from MQTT for which bin to get
        void handleBegin(); // handle processing the start message
        
        void sendBinData(int16_t binId, float weight, float cost); // send messages to be parsed by MQTT
};