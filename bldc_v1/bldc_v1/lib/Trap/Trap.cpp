#include "Trap.h"

Trap::Trap() {}

Trap::~Trap() {}

void Trap::run()
{
    if(newCycle){
        uint8_t hall = hallState;
        uint16_t throttle = throttleNormVal;
        writeTrap(hall, throttle);
        validateRpm();
        estimatePosition();
        Serial.print("v: ");
        Serial.print(rpm, 6);
        Serial.print("\t");
        Serial.print("p: ");
        Serial.println(rotorPos, 6);
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
        #ifdef SERIAL_DEBUG
            Serial.print(hall);
            Serial.print("\t");
            Serial.print((int)phaseA.mode);
            Serial.print((int)phaseB.mode);
            Serial.print((int)phaseC.mode);
        #endif
    }
    #ifdef SERIAL_DEBUG_CURRENTS
        if (newCurrentA)
        {
            Serial.print("\t");
            Serial.print(currentA);
            newCurrentA = false;
        }
        if (newCurrentB)
        {
            Serial.print("\t");
            Serial.print(currentB);
            newCurrentB = false;
        }
        if (newCurrentC)
        {
            Serial.print("\t");
            Serial.println(currentC);
            newCurrentC = false;
            Serial.print("\n");
        }
    #endif
    
}

void Trap::nextStep(uint8_t &currentHallState) {
    /**/
}

void Trap::writeTrap(uint8_t &halls, uint16_t uDuty){
    toggleLed();
    // Bound duty
    #ifdef SERIAL_DEBUG
        Serial.print("   ");
        Serial.print(uDuty);
        Serial.print("   ");
    #endif
    if(uDuty > 4096){ 
        uDuty = 4096;
    }
    if(uDuty < 0){
        uDuty = 0;
    }
    int16_t duty = (int16_t) uDuty;

    #ifdef BACKWARDS
        switch(halls){
            case 5: // 101: 
                setPhaseDuty(0, -1, duty);
                break;
            case 4: // 100:
                setPhaseDuty(duty, -1, 0);
                break;
            case 6: // 110:
                setPhaseDuty(duty, 0, -1);
                break;
            case 2: // 010:
                setPhaseDuty(0, duty, -1);
                break;
            case 3: // 011:
                setPhaseDuty(-1, duty, 0);
                break;
            case 1: // 001:
                setPhaseDuty(-1, 0, duty);
                break;
            default: // Undefined hall state
                setPhaseDuty(0, 0, 0);
        }

    #else
        switch(halls){
            case 5: // 101: 
                setPhaseDuty(0, duty, -1);
                break;
            case 4: // 100:
                setPhaseDuty(-1, duty, 0);
                break;
            case 6: // 110:
                setPhaseDuty(-1, 0, duty);
                break;
            case 2: // 010: 
                setPhaseDuty(0, -1, duty);
                break;
            case 3: // 011: 
                setPhaseDuty(duty, -1, 0);
                break;
            case 1: // 001: 
                setPhaseDuty(duty, 0, -1);
                break;
            default: // Undefined hall state
                setPhaseDuty(0, 0, 0);
        }
    #endif
    
    toggleLed();
}