#include "Bldc.h"

Bldc::Bldc() : controlType(Trap), hall_state(0), trap_duty(0) {}
Bldc::~Bldc() {}

void Bldc::driverInit() {
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(IRQ_FLAG_PIN, OUTPUT);
    analogReadResolution(ANALOG_RESOLUTION);
    analogReadAveraging(ANALOG_AVERAGING);
    pinMode(HALL_A_PIN, INPUT);
    pinMode(HALL_B_PIN, INPUT);
    pinMode(HALL_C_PIN, INPUT);
    configurePWMs();
    Serial.begin(115200);
    Serial.println("init");

    attachInterruptVector(IRQ_PWM, pwmIRQ_handler);
    NVIC_ENABLE_IRQ(IRQ_PWM);
}

void Bldc::configurePWMs() {
    // Enable PWM clocks
    CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON) | CCM_CCGR4_PWM2(CCM_CCGR_ON) |
                 CCM_CCGR4_PWM3(CCM_CCGR_ON) | CCM_CCGR4_PWM4(CCM_CCGR_ON);

    uint8_t used_submodules[] = {0, 1};  // FlexPWM4.0, FlexPWM4.1
    uint8_t used_submodules_flexpwm2[] = {0, 3}; // FlexPWM2.0, FlexPWM2.3
    uint8_t used_submodules_flexpwm1[] = {2, 3}; // FlexPWM1.2, FlexPWM1.3

    for (uint8_t i : used_submodules) {
        IMXRT_FLEXPWM4.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
        IMXRT_FLEXPWM4.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
        
        IMXRT_FLEXPWM4.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB; 
        
        IMXRT_FLEXPWM4.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << i) | FLEXPWM_OUTEN_PWMB_EN(1 << i);
        
        IMXRT_FLEXPWM4.SM[i].DTCNT0 = 200;  // Dead-time for complementary switching
        IMXRT_FLEXPWM4.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
        IMXRT_FLEXPWM4.SM[i].VAL0 = 0;
        IMXRT_FLEXPWM4.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
        IMXRT_FLEXPWM4.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
        IMXRT_FLEXPWM4.SM[i].VAL3 = (PWM_FREQUENCY / 4);
        IMXRT_FLEXPWM4.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
        IMXRT_FLEXPWM4.SM[i].VAL5 = (PWM_FREQUENCY / 8);
    }

    for (uint8_t i : used_submodules_flexpwm2) {
        IMXRT_FLEXPWM2.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
        IMXRT_FLEXPWM2.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
        
        IMXRT_FLEXPWM2.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB;
        
        IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << i) | FLEXPWM_OUTEN_PWMB_EN(1 << i);
        
        IMXRT_FLEXPWM2.SM[i].DTCNT0 = 200;
        IMXRT_FLEXPWM2.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
        IMXRT_FLEXPWM2.SM[i].VAL0 = 0;
        IMXRT_FLEXPWM2.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
        IMXRT_FLEXPWM2.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
        IMXRT_FLEXPWM2.SM[i].VAL3 = (PWM_FREQUENCY / 4);
        IMXRT_FLEXPWM2.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
        IMXRT_FLEXPWM2.SM[i].VAL5 = (PWM_FREQUENCY / 8);
    }

    for (uint8_t i : used_submodules_flexpwm1) {
        IMXRT_FLEXPWM1.SM[i].CTRL2 =  FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | FLEXPWM_SMCTRL_COMPMODE;
        IMXRT_FLEXPWM1.SM[i].CTRL = FLEXPWM_SMCTRL_HALF;
        IMXRT_FLEXPWM1.SM[i].CTRL2 &= ~(1 << 13);
        
        IMXRT_FLEXPWM1.SM[i].OCTRL = FLEXPWM_SMOCTRL_POLB  | FLEXPWM_SMOCTRL_POLA;
        
        IMXRT_FLEXPWM1.OUTEN |= (1 << (i * 2)) | (1 << (i * 2 + 1));

        
        IMXRT_FLEXPWM1.SM[i].DTCNT0 = 200;
        IMXRT_FLEXPWM1.SM[i].INIT = -((PWM_FREQUENCY / 2) - 1);
        IMXRT_FLEXPWM1.SM[i].VAL0 = 0;
        IMXRT_FLEXPWM1.SM[i].VAL1 = (PWM_FREQUENCY / 2) - 1;
        IMXRT_FLEXPWM1.SM[i].VAL2 = -((PWM_FREQUENCY / 4));
        IMXRT_FLEXPWM1.SM[i].VAL3 = (PWM_FREQUENCY / 4);
        IMXRT_FLEXPWM1.SM[i].VAL4 = -((PWM_FREQUENCY / 8));
        IMXRT_FLEXPWM1.SM[i].VAL5 = (PWM_FREQUENCY / 8);
    }

    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_LDOK(3); // Only submodules 0 and 1
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(9); // Only submodules 0 and 3
    IMXRT_FLEXPWM1.MCTRL |= FLEXPWM_MCTRL_LDOK(12); // Only submodules 2 and 3

    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_RUN(3);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(9);
    IMXRT_FLEXPWM1.MCTRL |= FLEXPWM_MCTRL_RUN(12);


    delay(10);
    setGatePWM(GATE_AH, 101);
    setGatePWM(GATE_AL, 101);
    setGatePWM(GATE_BH, 101);
    setGatePWM(GATE_BL, 101);
    setGatePWM(GATE_CH, 101);
    setGatePWM(GATE_CL, 101);
}




void Bldc::identifyHalls(uint8_t &current_hall_state){
    uint8_t aux = ((digitalRead(HALL_A_PIN) << 2) | (digitalRead(HALL_B_PIN) << 1) | (digitalRead(HALL_C_PIN)));
    if(aux == 0 || aux == 7) return;  // Discard invalid positions
    current_hall_state = aux;
}

void Bldc::getHalls(uint8_t &hall){
    uint8_t hallCounts[] = {0, 0, 0};

    for (uint8_t i = 0; i < HALL_OVERSAMPLE; i++) {
        hallCounts[0] += digitalRead(HALL_A_PIN);
        hallCounts[1] += digitalRead(HALL_B_PIN);
        hallCounts[2] += digitalRead(HALL_C_PIN);
    }
    hall = (hallCounts[0] > HALL_OVERSAMPLE / 2) << 0 |
           (hallCounts[1] > HALL_OVERSAMPLE / 2) << 1 |
           (hallCounts[2] > HALL_OVERSAMPLE / 2) << 2;

    Serial.print(hallCounts[2] > HALL_OVERSAMPLE / 2);
    Serial.print(hallCounts[1] > HALL_OVERSAMPLE / 2);
    Serial.println(hallCounts[0] > HALL_OVERSAMPLE / 2);
}

void Bldc::setPhaseDuty(uint16_t h_a, uint16_t l_a, uint16_t h_b, uint16_t l_b, uint16_t h_c, uint16_t l_c){
    setGatePWM(GATE_AH, h_a);
    setGatePWM(GATE_AL, l_a);
    setGatePWM(GATE_BH, h_b);
    setGatePWM(GATE_BL, l_b);
    setGatePWM(GATE_CH, h_c);
    setGatePWM(GATE_CL, l_c);
}

void Bldc::readThrottle(uint16_t &throttle){
    uint16_t throttle_raw = analogRead(THROTTLE_PIN);
    throttle_raw = (throttle_raw - THROTTLE_LOW) * 256 / (THROTTLE_HIGH - THROTTLE_LOW);
    throttle_raw = constrain(throttle_raw, 0, 255);
    throttle = (throttle_raw < 15) ? 0 : throttle_raw;
}

void Bldc::readCurrents(uint16_t &currentA, uint16_t &currentB, uint16_t &currentC){
    currentA = analogRead(CURRENT_SENSE_A);
    currentB = analogRead(CURRENT_SENSE_B);
    currentC = analogRead(CURRENT_SENSE_C);
}

void Bldc::setGatePWM(int gate, uint16_t pwm){
    analogWrite(gate, pwm);
}

void Bldc::pwmIRQ_handler() {
        digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    if (FLEXPWM_MODULE.SM[SUBMODULE].STS & FLEXPWM_SMSTS_RF) {
        FLEXPWM_MODULE.SM[SUBMODULE].STS = FLEXPWM_SMSTS_RF;
    }
}

void Bldc::run(){
    if (controlType == Trap) {
        // Implement trapezoidal control
    } else if (controlType == Foc) {
        // Implement FOC control
    }
}
