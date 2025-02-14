// #ifndef BLDC_H
// #define BLDC_H

// #include <Arduino.h>
// #include "Pinout.h"

// class Bldc
// {
// private:
//     uint16_t throttle_raw; // Analog values 
//     uint8_t hall_state; // Hall state 
//     // Time auxiliaries 
//     uint32_t blink_aux;
//     uint32_t throttle_aux;
//     uint32_t print_aux;  // Debug
//     static void identifyHalls(uint8_t &curent_hall_state);
//     static void setPhaseDuty(uint16_t &h_a, uint16_t &l_a, uint16_t &h_b, uint16_t &l_b, uint16_t &h_c, uint16_t &l_c);
//     static void readThrottle(uint16_t &throttle);
//     static void setGatePWM(int gate, uint16_t pwm);
// public:
//     Bldc();
//     ~Bldc();
//     static void driverInit(void);   // Initialize the driver
//     static void runTrapezoidAlgo(void);
// };

// #endif //BLDC_H
