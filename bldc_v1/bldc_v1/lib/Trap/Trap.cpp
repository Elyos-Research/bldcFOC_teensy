#include "Trap.h"

Trap::Trap() {}

Trap::~Trap() {}

void Trap::run()
{
    if(newCycle){
        uint8_t hall = hallState;
        uint16_t throttle = throttleNormVal;
        writeTrap(hall, throttle);
        newCycle = false;
    }
    if(newThrottleVal){
        uint8_t hall = 0;
        uint16_t throttle = static_cast<uint16_t>(throttleRawVal);
        readHalls(hall);
        normThrottle(throttle);
        throttleNormVal = throttle;
        hallState = hall;
        newThrottleVal = false;
        Serial.print(hall);
        Serial.print("    ");
        Serial.print((int)phaseA.mode);
        Serial.print((int)phaseB.mode);
        Serial.println((int)phaseC.mode);

    }
}

void Trap::writeTrap(uint8_t &halls, uint16_t uDuty){
    toggleLed();
    // Bound duty
    if(uDuty > 4096){ 
        uDuty = 4096;
    }
    if(uDuty < 0){
        uDuty = 1;
    }
    int16_t duty = (int16_t) uDuty;
    duty = 1;
    switch(halls){
        case 1: // Case 001
            setPhaseDuty(duty, 0, -1);
            break;
        case 2: // Case 010
            setPhaseDuty(0, -1, duty);
           break;
        case 3: // Case 011
            setPhaseDuty(duty, -1, 0);
            break;
        case 4: // Case 100
            setPhaseDuty(-1, duty, 0);
           break;
        case 5: // Case 101
            setPhaseDuty(0, duty, -1);
            break;
        case 6: // Case 110
            setPhaseDuty(-1, 0, duty);
            break;
        default: // Case 000 or error 
            setPhaseDuty(0, 0, 0);
    }
    toggleLed();
}