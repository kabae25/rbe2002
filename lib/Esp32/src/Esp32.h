#pragma once

class Esp32 {
    public:
        /**
         * init
         * Declare Serial1 to listen to
         * 
         */
        void init(void);

        /**
         * sendMessage creates a string of the form
         *      topic:message
         * which is what the corresponding ESP32 code expects.
         * */
        void sendMessage(const String& topic, const String& message);

        /**
         * 
         */
        void listenUART(void);

    private:
        bool checkSerial1(void);
};