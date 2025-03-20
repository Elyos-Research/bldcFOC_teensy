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


    // RPM measures
    const unsigned long RPM_TIMEOUT_US = 50000;
    volatile float rpmBuffer[FILTER_SIZE] = {0};
    volatile int rpmBufferIndex;
    volatile int rpmBufferCount;
    volatile unsigned long lastHallChangeTime;
    volatile float rpm;

    // Position measures
    volatile bool newHallUpdate;   
    volatile unsigned long lastPosUpdateTime; 
    volatile float rotorPos; 
    volatile uint8_t hallState;    // Current Hall sensor state

    enum class ControlType { Trap, Foc }; 
    enum class CurrentSensorChannel { A, B, C };

    // Control
    volatile unsigned long lastVelPosCalc;
    
    // Member variables
    volatile uint32_t throttleRawVal; // Throttle value
    volatile uint16_t throttleNormVal; // Throttle value

    // Global state variable to track the current sensor
    CurrentSensorChannel currentChannel = CurrentSensorChannel::A;
    
    // Currents measures
    volatile uint16_t currentA;
    volatile uint16_t currentB;
    volatile uint16_t currentC;
    volatile uint8_t newCurrentA;
    volatile uint8_t newCurrentB;
    volatile uint8_t newCurrentC;
    
    // Throttle
    volatile bool newThrottleVal;

    // PWM
    volatile bool newCycle;
    
    Bldc();
    ~Bldc();

    void driverInit(); // Initialize the driver
    virtual void run(); // Main loop function
    void readHalls(uint8_t &hallState);

    typedef struct {
        double Kp;        // Proportional gain
        double Ki;        // Integral gain
        double Kd;        // Derivative gain
        double prevError; // Previous error value
        double integral;  // Integral of error
    } PIDController_t;
    
    PIDController_t pid_vel;

    double computePID(PIDController_t &pid, double setpoint, double measurement, double dt);

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
    float estimatePosition();
    float hallToAngle(uint8_t hall);
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