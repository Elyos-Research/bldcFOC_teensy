

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
#define GATE_AH  2   // FlexPWM4.2 (A)
#define GATE_AL  3   // FlexPWM4.2 (B)
#define GATE_BH  28  // FlexPWM3.1 (A)
#define GATE_BL  29  // FlexPWM3.1 (B)
#define GATE_CH  36  // FlexPWM2.3 (A)
#define GATE_CL  37  // FlexPWM2.3 (B)

#define IRQ_PWM4_SUB0 90 // Teensy 4.1 uses IRQ #90 for FlexPWM4_0
#define IRQ_FLAG_PIN 4 //< Irq flag 

// ADC Inputs
#define THROTTLE_PIN    A13 
#define CURRENT_SENSE_A A14 // ADC
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
#define THROTTLE_HIGH 4094
#define THROTTLE_RESOLUTION 4095

#define PWM_FREQUENCY 20000 // 20 kHz
#define FLEXPWM_MODULE IMXRT_FLEXPWM4 // Using FLEXPWM4
#define SUBMODULE 2 // Using Submodule 0 (for GATE_AH)
#define IRQ_PWM IRQ_FLEXPWM4_2 // NVIC IRQ for FlexPWM4 Submodule 2

// #define 


#endif






















// void Bldc::configurePWMs() {

//     uint8_t used_submodules[] = {2, 1, 3};  // FlexPWM4.0, FlexPWM4.1
//     uint8_t used_submodules_flexpwm2[] = {0, 3}; // FlexPWM2.0, FlexPWM2.3
//     uint8_t used_submodules_flexpwm1[] = {2, 3}; // FlexPWM1.2, FlexPWM1.3

//     for (uint8_t i : used_submodules) {
//         IMXRT_FLEXPWM4.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
//         IMXRT_FLEXPWM4.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
        
//         IMXRT_FLEXPWM4.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB; 
        
//         IMXRT_FLEXPWM4.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << i) | FLEXPWM_OUTEN_PWMB_EN(1 << i);
        
//         IMXRT_FLEXPWM4.SM[i].DTCNT0 = 200;  // Dead-time for complementary switching
//         IMXRT_FLEXPWM4.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
//         IMXRT_FLEXPWM4.SM[i].VAL0 = 0;
//         IMXRT_FLEXPWM4.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
//         IMXRT_FLEXPWM4.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
//         IMXRT_FLEXPWM4.SM[i].VAL3 = (PWM_FREQUENCY / 4);
//         IMXRT_FLEXPWM4.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
//         IMXRT_FLEXPWM4.SM[i].VAL5 = (PWM_FREQUENCY / 8);
//     }

//     for (uint8_t i : used_submodules_flexpwm2) {
//         IMXRT_FLEXPWM2.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
//         IMXRT_FLEXPWM2.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
        
//         IMXRT_FLEXPWM2.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB;
        
//         IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << i) | FLEXPWM_OUTEN_PWMB_EN(1 << i);
        
//         IMXRT_FLEXPWM2.SM[i].DTCNT0 = 200;
//         IMXRT_FLEXPWM2.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
//         IMXRT_FLEXPWM2.SM[i].VAL0 = 0;
//         IMXRT_FLEXPWM2.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
//         IMXRT_FLEXPWM2.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
//         IMXRT_FLEXPWM2.SM[i].VAL3 = (PWM_FREQUENCY / 4);
//         IMXRT_FLEXPWM2.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
//         IMXRT_FLEXPWM2.SM[i].VAL5 = (PWM_FREQUENCY / 8);
//     }

//     for (uint8_t i : used_submodules_flexpwm1) {
//         IMXRT_FLEXPWM1.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | FLEXPWM_SMCTRL_COMPMODE;
//         IMXRT_FLEXPWM1.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
//         IMXRT_FLEXPWM1.SM[i].CTRL2 &= ~(1 << 13);
        
//         IMXRT_FLEXPWM1.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB;
        
//         IMXRT_FLEXPWM1.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << i) | FLEXPWM_OUTEN_PWMB_EN(1 << i);
        
//         IMXRT_FLEXPWM1.SM[i].DTCNT0 = 200;
//         IMXRT_FLEXPWM1.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
//         IMXRT_FLEXPWM1.SM[i].VAL0 = 0;
//         IMXRT_FLEXPWM1.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
//         IMXRT_FLEXPWM1.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
//         IMXRT_FLEXPWM1.SM[i].VAL3 = (PWM_FREQUENCY / 4);
//         IMXRT_FLEXPWM1.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
//         IMXRT_FLEXPWM1.SM[i].VAL5 = (PWM_FREQUENCY / 8);
//     }

//     IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_LDOK(3); // Only submodules 0 and 1
//     IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(9); // Only submodules 0 and 3
//     IMXRT_FLEXPWM1.MCTRL |= FLEXPWM_MCTRL_LDOK(12); // Only submodules 2 and 3

//     IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_RUN(3);
//     IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(9);
//     IMXRT_FLEXPWM1.MCTRL |= FLEXPWM_MCTRL_RUN(12);


//     delay(10);
//     setGatePWM(GATE_AH, 101);
//     setGatePWM(GATE_AL, 101);
//     setGatePWM(GATE_BH, 101);
//     setGatePWM(GATE_BL, 101);
//     setGatePWM(GATE_CH, 101);
//     setGatePWM(GATE_CL, 101);
// }

