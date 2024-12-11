#include <Arduino.h>
#include <Esp32.h>

void Esp32::init(void) {
    Serial1.begin(115200); // esp UART
    digitalWrite(0, HIGH); // Set internal pullup on RX1 to avoid spurious signals

    Serial1.println("/setup()");
    Serial.println("Esp32 Connected");
}

void Esp32::sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}

String serString1;
bool Esp32::checkSerial1(void)
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}

void Esp32::listenUART(void) {
    #ifdef __ESP_DEBUG__
    static uint32_t lastSend = 0;
    uint32_t currTime = millis();
    if(currTime - lastSend >= 5000) //send every five seconds
    {
        lastSend = currTime;
        sendMessage("evan/heartbeat", String(currTime));
        Serial.println("Sending heartbeat to: evan/heartbeat");
    }
    #endif


    // Check to see if we've received anything
    if(checkSerial1())
    {
        Serial.print("Rec'd:\t");
        Serial.print(serString1);
        serString1 = "";
    }
}