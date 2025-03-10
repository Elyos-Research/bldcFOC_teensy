#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>
#include "Pinout.h"
#include <cstdint>

class Bldc {
public:
    enum class ControlType { Trap, Foc }; 
    
    // Member variables
    volatile uint32_t throttleRawVal; // Throttle value
    volatile uint16_t throttleNormVal; // Throttle value
    volatile uint8_t hallState;    // Current Hall sensor state
    volatile bool newCycle;
    volatile bool newThrottleVal;

    
    Bldc();
    ~Bldc();

    void driverInit(); // Initialize the driver
    virtual void run(); // Main loop function

protected:
    // Phase structure
    struct Phase {
        enum class Mode : uint8_t {
        Complementary = 0,
        X = 1,
        ZeroComplement = 2
        };
        
        uint16_t pwmVal;  // PWM value for the high side phase
        uint8_t highSide; // High-side gate pin
        uint8_t lowSide;  // Low-side gate pin
        Mode mode;        // Operating mode
        uint8_t phaseID;  // A, B, C - 1, 2, 3
    };

    // Objects
    ControlType controlType; // Control type (Trap or Foc)

    Phase phaseA; // Phase A configuration
    Phase phaseB; // Phase B configuration
    Phase phaseC; // Phase C configuration

    // Configuration
    void configurePWM();
    void configureADC();

    // Hall sensors
    void readHalls(uint8_t &hallState);
    void identifyHalls(uint8_t &currentHallState);

    // PWM Control
    void setGatePWM(Phase phase);
    void setPhaseDuty(uint16_t phaseApwm, uint16_t phaseBpwm, uint16_t phaseCpwm);

    // Sensor readings
    void normThrottle(uint16_t &throttle);
    void readCurrents(uint16_t &currentA, uint16_t &currentB, uint16_t &currentC);

    // PWM utilities
    void setPwmFrequency(IMXRT_FLEXPWM_t *pwmModule, uint8_t submodule, uint8_t channel, float freq);
    void writePwmValue(IMXRT_FLEXPWM_t *pwmModule, uint8_t submodule, uint8_t channel, uint16_t value, Phase::Mode mode);

};

// ADC IRQ trigger
void triggerADC();

// Debug utilities
void toggleLed();
void toggleFlag();

#endif // BLDC_H