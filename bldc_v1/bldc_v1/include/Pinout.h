

/*
   PWM Pin Configuration

   | PWM Pin |    Timer     | Default Frequency | Desired Frequency |
   |---------|-------------|-------------------|-------------------|
   |   22    | FlexPWM4.0  |      4.482 kHz    |       20 kHz      |
   |   23    | FlexPWM4.1  |      4.482 kHz    |       20 kHz      |
   |   33    | FlexPWM2.0  |      4.482 kHz    |       20 kHz      |
   |   37    | FlexPWM2.3  |      4.482 kHz    |       20 kHz      |
   |   24    | FlexPWM1.2  |      4.482 kHz    |       20 kHz      |
   |   25    | FlexPWM1.3  |      4.482 kHz    |       20 kHz      |
*/



//////////////////////////////////////////////////
//////////////// MACRO DEFINITION ////////////////
//////////////////////////////////////////////////

#ifndef PINOUT_H
#define PINOUT_H

// HALL Sensors
#define HALL_A_PIN 12
#define HALL_B_PIN 11
#define HALL_C_PIN 10

#define HALL_OVERSAMPLE ((uint8_t)8)

// DRIVER GATES (PWM Outputs)
#define GATE_AH  22 
#define GATE_AL  23 
#define GATE_BH  33 
#define GATE_BL  37 
#define GATE_CH  24  //< Cambio
#define GATE_CL  25  //< Cambio

#define IRQ_PWM4_SUB0 90 // Teensy 4.1 uses IRQ #90 for FlexPWM4_0
#define IRQ_FLAG_PIN 4 //< Irq flag 

// ADC Inputs
#define THROTTLE_PIN    A13 
#define CURRENT_SENSE_A A14 //< Cambio
#define CURRENT_SENSE_B A15 //< Cambio
#define CURRENT_SENSE_C A16 //< Cambio

// ADC Resolution & Sampling
#define ANALOG_RESOLUTION 12
#define ANALOG_AVERAGING  8

// Other Pins
#define BUILTIN_LED_PIN 13

// Time Control Macros (in ms)
#define TIME_TO_READ_THROTTLE 500
#define TIME_TO_BLINK 500
#define TIME_TO_PRINT 1000

#define THROTTLE_LOW 10
#define THROTTLE_HIGH 250






#define PWM_FREQUENCY 20000 // 20 kHz
#define FLEXPWM_MODULE IMXRT_FLEXPWM4 // Using FLEXPWM4
#define SUBMODULE 0 // Using Submodule 0 (for GATE_AH)
#define IRQ_PWM IRQ_FLEXPWM4_0 // NVIC IRQ for FlexPWM4 Submodule 0



#endif
