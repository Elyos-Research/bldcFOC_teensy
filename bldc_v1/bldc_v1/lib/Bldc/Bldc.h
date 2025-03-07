#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>
#include "Pinout.h"

class Bldc
{
private:
    uint32_t trap_duty;
    uint8_t hall_state;

    uint8_t current_sense_index;
    
    // Configuration
    void configurePWMs();
    void configureADCs();
    // PWM Control
    void setGatePWM(int gate, int pwm);
    void setPhaseDuty(uint16_t h_a, uint16_t l_a, uint16_t h_b, uint16_t l_b, uint16_t h_c, uint16_t l_c);
    // Hall sensors
    void getHalls(uint8_t &hall);
    void identifyHalls(uint8_t &current_hall_state);
    // Read sensors
    void readThrottle(uint16_t &throttle);
    void readCurrents(uint16_t &currentA, uint16_t &currentB, uint16_t &currentC);

    void flexpwmFrequencyCA(IMXRT_FLEXPWM_t*p, unsigned int submodule, uint8_t channel, float freq);
    void flexpwmWriteCA( IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, uint16_t val1);
public:
    enum ControlType { Trap, Foc };
    ControlType controlType;

    Bldc();
    ~Bldc();
    
    void driverInit();
    void run();
    static void adcISR();

};

#endif //BLDC_H
