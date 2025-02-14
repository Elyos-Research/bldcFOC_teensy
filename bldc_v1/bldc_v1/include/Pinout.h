
//////////////////////////////////////////////////
//////////////// MACRO DEFINITION ////////////////
//////////////////////////////////////////////////

#ifndef PINOUT_H
#define PINOUT_H

// HALL Sensors
#define HALL_A_PIN 10
#define HALL_B_PIN 11
#define HALL_C_PIN 12

// DRIVER GATES (PWM Outputs)
#define GATE_AH  5   // PWM capable pin
#define GATE_AL  6   // PWM capable pin
#define GATE_BH  9   // PWM capable pin
#define GATE_BL  10  // PWM capable pin
#define GATE_CH  20  // PWM capable pin
#define GATE_CL  21  // PWM capable pin

// ADC Inputs
#define THROTTLE_PIN    A13
#define CURRENT_SENSE_A A12
#define CURRENT_SENSE_B A11
#define CURRENT_SENSE_C A10

// ADC Resolution & Sampling
#define ANALOG_RESOLUTION 12
#define ANALOG_AVERAGING  8

// Other Pins
#define BUILTIN_LED_PIN 13

// Time Control Macros (in ms)
#define TIME_TO_READ_THROTTLE 500
#define TIME_TO_BLINK 500
#define TIME_TO_PRINT 1000

#endif
