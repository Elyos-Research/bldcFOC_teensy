#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>
#include "Pinout.h"
#include <cstdint>

class Bldc {
public:
    #ifdef MEASURE_TICKS_PER_REV
        int testRev;
    #endif

    float rpmBuffer[FILTER_SIZE] = {0};
    int rpmBufferIndex;
    int rpmBufferCount;
    unsigned long lastHallChangeTime;
    const unsigned long RPM_TIMEOUT_MS = 50;
    enum class ControlType { Trap, Foc }; 
    enum class CurrentSensorChannel { A, B, C };

    // Global state variable to track the current sensor
    CurrentSensorChannel currentChannel = CurrentSensorChannel::A;
    
    // Member variables
    volatile uint32_t throttleRawVal; // Throttle value
    volatile uint16_t throttleNormVal; // Throttle value
    volatile uint16_t currentA;
    volatile uint16_t currentB;
    volatile uint16_t currentC;
    volatile float rpm;
    volatile uint8_t hallState;    // Current Hall sensor state
    volatile bool newCycle;
    volatile bool newThrottleVal;
    volatile uint8_t newCurrentA;
    volatile uint8_t newCurrentB;
    volatile uint8_t newCurrentC;
    
    Bldc();
    ~Bldc();

    void driverInit(); // Initialize the driver
    virtual void run(); // Main loop function

protected:
    // Phase structure
    struct Phase {  //< TODO: recibe pwm uinsigned pero debe de ser consigno!
        enum class Mode : uint8_t {
        Complementary = 0,
        X = 1,
        ZeroComplement = 2
        };
        
        int16_t pwmVal;  // PWM value for the high side phase
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
    virtual void nextStep(uint8_t &currentHallState);

    // PWM Control
    void setGatePWM(Phase phase);
    void setPhaseDuty(int16_t phaseApwm, int16_t phaseBpwm, int16_t phaseCpwm);

    // Sensor readings
    void normThrottle(uint16_t &throttle);
    void readCurrents(int16_t &currentA, int16_t &currentB, int16_t &currentC);

    // PWM utilities
    void setPwmFrequency(IMXRT_FLEXPWM_t *pwmModule, uint8_t submodule, uint8_t channel, float freq);
    void writePwmValue(IMXRT_FLEXPWM_t *pwmModule, uint8_t submodule, uint8_t channel, int16_t value, Phase::Mode mode);

    void validateRpm();
};

// ADC IRQ trigger
void triggerADC();
void triggerADC_CurrentA();
void triggerADC_CurrentB();
void triggerADC_CurrentC();

// Debug utilities
void toggleLed();
void toggleFlag();

#endif // BLDC_H