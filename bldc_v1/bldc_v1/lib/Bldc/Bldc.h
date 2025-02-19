#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>
#include "Pinout.h"

class Bldc
{
private:
    uint32_t trap_duty;
    uint8_t hall_state;

    static void getHalls(uint8_t &hall);
    static void readThrottle(uint16_t &throttle);
    void identifyHalls(uint8_t &current_hall_state);
    void setPhaseDuty(uint16_t h_a, uint16_t l_a, uint16_t h_b, uint16_t l_b, uint16_t h_c, uint16_t l_c);
    static void readCurrents(uint16_t &currentA, uint16_t &currentB, uint16_t &currentC);
    void setGatePWM(int gate, uint16_t pwm);
    void configurePWMs();

public:
    enum ControlType { Trap, Foc };
    ControlType controlType;

    Bldc();
    ~Bldc();
    void driverInit();
    void run();
    static void pwmReloadISR();

};

#endif //BLDC_H
