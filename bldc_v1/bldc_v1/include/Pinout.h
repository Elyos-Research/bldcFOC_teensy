

/*
   PWM Pin Configuration

   | PWM Pin |    Timer     | Default Frequency | Desired Frequency |
   |---------|-------------|-------------------|-------------------|
   |   2     | FlexPWM4.2  |      4.482 kHz    |       20 kHz      |
   |   3     | FlexPWM4.2  |      4.482 kHz    |       20 kHz      |
   |   28    | FlexPWM3.1  |      4.482 kHz    |       20 kHz      |
   |   29    | FlexPWM3.1  |      4.482 kHz    |       20 kHz      |
   |   36    | FlexPWM2.3  |      4.482 kHz    |       20 kHz      |
   |   37    | FlexPWM2.3  |      4.482 kHz    |       20 kHz      |
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
#define GATE_AH  4   // FlexPWM2.0 (A)
#define GATE_AL  33  // FlexPWM2.0 (B)
#define GATE_BH  6   // FlexPWM2.2 (A)
#define GATE_BL  9   // FlexPWM2.2 (B)
#define GATE_CH  36  // FlexPWM2.3 (A)
#define GATE_CL  37  // FlexPWM2.3 (B)

#define IRQ_FLAG_PIN 4 //< Irq flag 

// ADC Inputs
#define THROTTLE_PIN    A10 // ADC1_1 = A10 = AD_B0_12  ADC1 channel 1     ADC1_HC0 = ADC_channel
#define CURRENT_SENSE_A A12 // ADC2_3 = A12 = AD_B1_14  ADC2 channel 131   ADC2_HC0 = ADC_channel & 0x7f
#define CURRENT_SENSE_B A13 // ADC2_4 = A13 = AD_B1_15  ADC2 channel 132   ADC2_HC0 = ADC_channel & 0x7f
#define CURRENT_SENSE_C A14 // ADC2_1 = A14 = AD_B1_12  ADC2 channel 129   ADC2_HC0 = ADC_channel & 0x7f

// ADC Resolution & Sampling
#define ANALOG_RESOLUTION 12
#define ANALOG_AVERAGING  8

// Other Pins
#define BUILTIN_LED_PIN 13
constexpr std::size_t kBuiltInLedPin{13};

// Time Control Macros (in ms)
#define TIME_TO_READ_THROTTLE 500
#define TIME_TO_BLINK 500
#define TIME_TO_PRINT 1000

#define THROTTLE_LOW 10
#define THROTTLE_HIGH 4094
#define THROTTLE_RESOLUTION 4095

#define PWM_FREQUENCY 20000 // 20 kHz
#define FLEXPWM_MODULE IMXRT_FLEXPWM4 // Using FLEXPWM4
#define SUBMODULE 2 // Using Submodule 0 (for GATE_AH)
#define IRQ_PWM IRQ_FLEXPWM4_2 // NVIC IRQ for FlexPWM4 Submodule 2

#endif













