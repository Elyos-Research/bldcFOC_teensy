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

    IMXRT_FLEXPWM4.SM[2].VAL0 = 0;
    IMXRT_FLEXPWM4.SM[2].VAL1 = 4095;
    IMXRT_FLEXPWM4.SM[2].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM4.SM[2].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM4.SM[2].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM4.SM[2].DTCNT0 = 70;
    IMXRT_FLEXPWM4.SM[2].DTCNT1 = 70;
    IMXRT_FLEXPWM4.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 4) | FLEXPWM_OUTEN_PWMB_EN(1 << 4);
    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_LDOK(4);
    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_RUN(4);


    IMXRT_FLEXPWM3.SM[1].VAL0 = 0;
    IMXRT_FLEXPWM3.SM[1].VAL1 = 4095;
    IMXRT_FLEXPWM3.SM[1].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM3.SM[1].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM3.SM[1].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM3.SM[1].DTCNT0 = 70;
    IMXRT_FLEXPWM3.SM[1].DTCNT1 = 70;
    IMXRT_FLEXPWM3.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2);
    IMXRT_FLEXPWM3.MCTRL |= FLEXPWM_MCTRL_LDOK(2);
    IMXRT_FLEXPWM3.MCTRL |= FLEXPWM_MCTRL_RUN(2);


    IMXRT_FLEXPWM2.SM[3].VAL0 = 0;
    IMXRT_FLEXPWM2.SM[3].VAL1 = 4095;
    IMXRT_FLEXPWM2.SM[3].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[3].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM2.SM[3].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[3].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[3].DTCNT1 = 70;
    IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 6) | FLEXPWM_OUTEN_PWMB_EN(1 << 6);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(6);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(6);


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
