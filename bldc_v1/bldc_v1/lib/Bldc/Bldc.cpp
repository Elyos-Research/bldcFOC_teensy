#include "Bldc.h"

extern "C" {
    extern  void xbar_connect(unsigned int input, unsigned int output);
}

////////////////////////////////////////////////////////////////////
///////////////////////// MAIN FUNCTIONS /////////////////////////// 
////////////////////////////////////////////////////////////////////

Bldc::Bldc() : controlType(Trap), hall_state(0), trap_duty(0) {}

Bldc::~Bldc() {}

void Bldc::driverInit() {
    // Halls 
    pinMode(HALL_A_PIN, INPUT);
    pinMode(HALL_B_PIN, INPUT);
    pinMode(HALL_C_PIN, INPUT);

    // Debug flags
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(IRQ_FLAG_PIN, OUTPUT);

    // Configuring
    configurePWMs();
    configureADCs();

    Serial.begin(115200);
    Serial.println("init");
}

void Bldc::run(){
    setGatePWM(GATE_AH, 150);
    setGatePWM(GATE_AL, 150);
    setGatePWM(GATE_BH, 150);
    setGatePWM(GATE_BL, 150);
    setGatePWM(GATE_CH, 150);
    setGatePWM(GATE_CL, 150);
}


////////////////////////////////////////////////////////////////////
////////////////////////// PWM FUNCTIONS ///////////////////////////
////////////////////////////////////////////////////////////////////

void PWM4_CompletedCallback(){
    // Reload PWM interrupt
    IMXRT_FLEXPWM4.SM[2].STS = 0x4;
    IMXRT_FLEXPWM4.SM[2].INTEN = 0x4;

    // Start measuring ADC's
    // IMXRT_ADC2.HC0 = ADC_HC_ADCH(3) | ADC_HC_ADCH(4) | ADC_HC_AIEN;
}

void Bldc::configurePWMs() {
    IMXRT_FLEXPWM4.SM[2].VAL0 = 0;
    IMXRT_FLEXPWM4.SM[2].VAL1 = 4095;
    IMXRT_FLEXPWM4.SM[2].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM4.SM[2].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM4.SM[2].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM4.SM[2].DTCNT0 = 70;
    IMXRT_FLEXPWM4.SM[2].DTCNT1 = 70;

    IMXRT_FLEXPWM3.SM[1].VAL0 = 0;
    IMXRT_FLEXPWM3.SM[1].VAL1 = 4095;
    IMXRT_FLEXPWM3.SM[1].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM3.SM[1].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM3.SM[1].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM3.SM[1].DTCNT0 = 70;
    IMXRT_FLEXPWM3.SM[1].DTCNT1 = 70;

    IMXRT_FLEXPWM2.SM[3].VAL0 = 0;
    IMXRT_FLEXPWM2.SM[3].VAL1 = 4095;
    IMXRT_FLEXPWM2.SM[3].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[3].CTRL |= FLEXPWM_SMCTRL_FULL;
    IMXRT_FLEXPWM2.SM[3].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[3].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[3].DTCNT1 = 70;
    
    IMXRT_FLEXPWM4.SM[2].STS = 0x4;
    IMXRT_FLEXPWM4.SM[2].INTEN = 0x4;
    
    IMXRT_FLEXPWM4.FCTRL0 |= FLEXPWM_FCTRL0_FLVL(4);
    IMXRT_FLEXPWM3.FCTRL0 |= FLEXPWM_FCTRL0_FLVL(2);
    IMXRT_FLEXPWM2.FCTRL0 |= FLEXPWM_FCTRL0_FLVL(6);

    IMXRT_FLEXPWM4.SM[2].CNT = 0;
    IMXRT_FLEXPWM3.SM[1].CNT = 0;
    IMXRT_FLEXPWM2.SM[3].CNT = 0;

    // Enable initialization by force in PWM4
    IMXRT_FLEXPWM4.SM[2].CTRL2 &= ~FLEXPWM_SMCTRL2_FRCEN;
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_FRCEN;
    IMXRT_FLEXPWM3.SM[1].CTRL2 |= FLEXPWM_SMCTRL2_FRCEN;

    // PWM4 configured to be the master 
    IMXRT_FLEXPWM4.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0);

    // Force select settings
    IMXRT_FLEXPWM4.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(0);
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);
    IMXRT_FLEXPWM3.SM[1].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);

    // PWM4 FORCE signal triggers synchronization
    IMXRT_FLEXPWM4.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;

    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_CLDOK(4);
    IMXRT_FLEXPWM3.MCTRL |= FLEXPWM_MCTRL_CLDOK(6);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_CLDOK(2);

    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_LDOK(4);
    IMXRT_FLEXPWM3.MCTRL |= FLEXPWM_MCTRL_LDOK(2);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(6);

    IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_RUN(4);
    IMXRT_FLEXPWM3.MCTRL |= FLEXPWM_MCTRL_RUN(2);
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(6);
    
    IMXRT_FLEXPWM4.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 4) | FLEXPWM_OUTEN_PWMB_EN(1 << 4);
    IMXRT_FLEXPWM3.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2);
    IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 6) | FLEXPWM_OUTEN_PWMB_EN(1 << 6);

    // Enable Hardware trigger for ADC conversions when VAL1 is reached (PWM_OUT_TRIG1)
    IMXRT_FLEXPWM4.SM[2].TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(1); 

    attachInterruptVector(IRQ_FLEXPWM4_2, PWM4_CompletedCallback);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_2);
    
    delay(1);
    setGatePWM(GATE_AH, 0);
    setGatePWM(GATE_AL, 0);
    setGatePWM(GATE_BH, 0);
    setGatePWM(GATE_BL, 0);
    setGatePWM(GATE_CH, 0);
    setGatePWM(GATE_CL, 0);
}

void Bldc::setGatePWM(int gate, uint16_t pwm){
    analogWrite(gate, pwm);
}

void Bldc::setPhaseDuty(uint16_t h_a, uint16_t l_a, uint16_t h_b, uint16_t l_b, uint16_t h_c, uint16_t l_c){
    setGatePWM(GATE_AH, h_a);
    setGatePWM(GATE_AL, l_a);
    setGatePWM(GATE_BH, h_b);
    setGatePWM(GATE_BL, l_b);
    setGatePWM(GATE_CH, h_c);
    setGatePWM(GATE_CL, l_c);
}


////////////////////////////////////////////////////////////////////
////////////////////////// ADC FUNCTIONS /////////////////////////// 
////////////////////////////////////////////////////////////////////

void ADC2_CompletedConversionCallback(){
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));

    // Get ADC value from register
    uint16_t val = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095;

    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));    

    ADC_ETC_DONE0_1_IRQ |= 1 << 16;  // Clear
}   

void Bldc::configureADCs(){
    // Use XBAR to connect PWM4.2 to ADC ETC 0
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
    xbar_connect(XBARA1_IN_FLEXPWM4_PWM2_OUT_TRIG1, XBARA1_OUT_ADC_ETC_TRIG01);

    // Configure ADC2 oversample and 12 bit conversion
    IMXRT_ADC2.CFG = ADC_CFG_AVGS(0b01) | ADC_CFG_MODE(0b10) | ADC_CFG_ADTRG;
    // Continuous conversion and enable averaging by hardware
    IMXRT_ADC2.GC = ADC_GC_AVGE;  // ADC_GC_ADCO | 

    // Register Interrupt 
    // attachInterruptVector(IRQ_ADC2, ADC2_CompletedConversionCallback);
    // NVIC_ENABLE_IRQ(IRQ_ADC2);

    // ADC External Trigger Controller (ETC)
    // Soft Reset the ADC_ETC module
    IMXRT_ADC_ETC.CTRL = ADC_ETC_CTRL_SOFTRST;  
    // Clear the reset
    IMXRT_ADC_ETC.CTRL &= ~ADC_ETC_CTRL_SOFTRST; 
    delay(5);
    // Enable External Signal Controller 1
    IMXRT_ADC_ETC.CTRL = ADC_ETC_CTRL_TRIG_ENABLE((1 << 1));

    IMXRT_ADC_ETC.TRIG[1].CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE0(1)  /* Enable interrupt for first conversion */
    | ADC_ETC_TRIG_CHAIN_HWTS0(1)  /* Select Hardware Trigger Source 0 */
    | ADC_ETC_TRIG_CHAIN_CSEL0(1)  /* First conversion: ADC2 channel 1 */
    ;

    // START ADC's
    IMXRT_ADC2.HC0 = ADC_HC_ADCH(1) | ADC_HC_AIEN;
    IMXRT_ADC2.HC1 = ADC_HC_ADCH(3) | ADC_HC_AIEN;
    IMXRT_ADC2.HC2 = ADC_HC_ADCH(4) | ADC_HC_AIEN;

    // Link external trigger interrupt to callback
    attachInterruptVector(IRQ_ADC_ETC0, ADC2_CompletedConversionCallback);
    NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
    attachInterruptVector(IRQ_ADC_ETC1, ADC2_CompletedConversionCallback);
    NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}

void Bldc::readThrottle(uint16_t &throttle){
    uint16_t throttle_raw = analogRead(THROTTLE_PIN);
    uint16_t lowBound = (throttle_raw - THROTTLE_LOW);
    if(lowBound < 0){ lowBound = 0;}
    throttle_raw = lowBound * (THROTTLE_HIGH + 1) / (THROTTLE_HIGH - THROTTLE_LOW);
    throttle_raw = lowBound * THROTTLE_HIGH / THROTTLE_RESOLUTION;
    if (throttle_raw > THROTTLE_RESOLUTION){ throttle_raw = 0;}    
    Serial.println(throttle_raw);
}

void Bldc::readCurrents(uint16_t &currentA, uint16_t &currentB, uint16_t &currentC){
    currentA = analogRead(CURRENT_SENSE_A);
    currentB = analogRead(CURRENT_SENSE_B);
    currentC = analogRead(CURRENT_SENSE_C);
}


////////////////////////////////////////////////////////////////////
////////////////////// HALL SENSOR HANDLING //////////////////////// 
////////////////////////////////////////////////////////////////////

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
}
