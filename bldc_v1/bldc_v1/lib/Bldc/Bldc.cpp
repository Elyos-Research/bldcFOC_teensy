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
  /*1-4000*/
    setGatePWM(GATE_AH, 3000);
    setGatePWM(GATE_AL, 3000);

    setGatePWM(GATE_BH, 1000);
    setGatePWM(GATE_BL, 1000);

    setGatePWM(GATE_CH, 100);
    setGatePWM(GATE_CL, 100);
}


////////////////////////////////////////////////////////////////////
////////////////////////// PWM FUNCTIONS ///////////////////////////
////////////////////////////////////////////////////////////////////

void PWM2_CompletedCallback(){
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    // Reload PWM interrupt
    IMXRT_FLEXPWM2.SM[0].STS = 0x1;
    IMXRT_FLEXPWM2.SM[0].INTEN = 0x1;

    // Start measuring ADC's
    // IMXRT_ADC2.HC0 = ADC_HC_ADCH(3) | ADC_HC_ADCH(4) | ADC_HC_AIEN;
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
}

void Bldc::flexpwmFrequencyCA( IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, float frequency)
{
    uint16_t mask     = 1 << submodule;
    uint16_t oldinit  = p->SM[submodule].INIT;
    uint32_t olddiv   = (int32_t)p->SM[submodule].VAL1 - (int32_t)oldinit;
    uint32_t newdiv   = (uint32_t)((float)F_BUS_ACTUAL / frequency + 0.5f);
    uint32_t prescale = 0;

    while (newdiv > 65535 && prescale < 7) {
      newdiv = newdiv >> 1;
      prescale = prescale + 1;
    }

    if (newdiv > 65535) {
      newdiv = 65535;
    } else if (newdiv < 2) {
      newdiv = 2;
    }
    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    p->SM[submodule].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);

    p->SM[submodule].INIT = -(newdiv/2);
    p->SM[submodule].VAL1 = +(newdiv/2);
    p->SM[submodule].VAL0 = 0;
    p->SM[submodule].VAL2 = 0;
    p->SM[submodule].VAL3 = 0;
    p->SM[submodule].VAL4 = 0;
    p->SM[submodule].VAL5 = 0;
    p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
    
    #define pV (p->SM[submodule])
}

void Bldc::flexpwmWriteCA( IMXRT_FLEXPWM_t *p, unsigned int submodule,
  uint8_t channel, uint16_t val1)
{
  // Temporal fix
  uint16_t val = 4095 - val1;
  uint16_t mask = 1 << submodule;
  uint32_t modulo = p->SM[submodule].VAL1 * 2; // - p->SM[submodule].INIT;
  uint32_t cval = ((uint32_t)val * (modulo + 1)) >> 12/* (pwm res is the last number) analog_write_res*/;
  if (cval > modulo) cval = modulo; // TODO: is this check correct?
    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    switch (channel) {
    case 0: // X
      p->SM[submodule].VAL0 = modulo - cval;
      p->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
      break;
    case 1: // A
      p->SM[submodule].VAL2 = -(cval/2);
      p->SM[submodule].VAL3 = +(cval/2);
      p->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
      break;
    case 2: // B
      p->SM[submodule].VAL4 = -(cval/2);
      p->SM[submodule].VAL5 = +(cval/2);
      p->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
  }
  p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
  #define portConfigRegister(pin)  ((digital_pin_to_info_PGM[(pin)].mux))
}

void Bldc::configurePWMs() {
    // Check Ref Manual pg 3074 //
    IMXRT_FLEXPWM2.SM[0].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[0].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[0].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[0].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[0].DTCNT1 = 70;

    IMXRT_FLEXPWM2.SM[2].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[2].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[2].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[2].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[2].DTCNT1 = 70;

    IMXRT_FLEXPWM2.SM[3].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[3].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[3].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[3].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[3].DTCNT1 = 70;
    
    // Enable interrupt for HALF (0x1) or FULL (0x2) CYCLE
    IMXRT_FLEXPWM2.SM[0].STS = 0x1;
    IMXRT_FLEXPWM2.SM[0].INTEN = 0x1;
    
    // Fault interrupt enable
    IMXRT_FLEXPWM2.FCTRL0 |= FLEXPWM_FCTRL0_FLVL(4);

    // PWM4 configured to be the master 
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b00);
    IMXRT_FLEXPWM2.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b10);
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b10);

    // Force select settings
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(0);
    IMXRT_FLEXPWM2.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);

    // PWM2 SM0 forces other submodules initialization for sync
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;

    // Load ready flags
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN((1<<0) | (1<<2) | (1<<3));
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_CLDOK((1<<0) | (1<<2) | (1<<3));
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK((1<<0) | (1<<2) | (1<<3));

    // Enable Hardware trigger for ADC conversions when VAL1 is reached (PWM_OUT_TRIG1)
    IMXRT_FLEXPWM2.SM[0].TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(0); 
    // Set PWM Interrupt (configured to mid cycle)
    attachInterruptVector(IRQ_FLEXPWM2_0, PWM2_CompletedCallback);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_0);

    // Set PWM frequencies 
    #define M(a, b) ((((a) - 1) << 4) | (b))
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, 30000);
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, 30000);
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, 30000);
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, 30000);
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, 30000);
    flexpwmFrequencyCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, 30000);

    // Write zeros to everything
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, 0);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, 0);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, 0);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, 0);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, 0);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, 0);

    // Configure signal propagation to pads
    #define portConfigRegister(pin)  ((digital_pin_to_info_PGM[(pin)].mux))
    *(portConfigRegister(4)) =  1;  // GATE AH
    *(portConfigRegister(33)) = 1;  // GATE AL
    *(portConfigRegister(6)) =  2;  // GATE BH
    *(portConfigRegister(9)) =  2;  // GATE BL
    *(portConfigRegister(36)) = 6;  // GATE CH
    *(portConfigRegister(37)) = 6;  // GATE CL
    
    //Enable outputs in A and B 
    IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 3) | FLEXPWM_OUTEN_PWMB_EN(1 << 3) 
                         | FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2)
                         | FLEXPWM_OUTEN_PWMA_EN(1) | FLEXPWM_OUTEN_PWMB_EN(1);

    delay(1);
}

void Bldc::setGatePWM(int gate, int pwm){
  if (gate == GATE_AH || gate == GATE_AL)
  {
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, pwm);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, pwm);
  }
  if (gate == GATE_BH || gate == GATE_BL)
  {
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, 500);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, 500);
  }
  if (gate == GATE_CH || gate == GATE_CL)
  {
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, 150);
    flexpwmWriteCA(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, 150);
  }
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
    // Clear ISR flag
    ADC_ETC_DONE0_1_IRQ |= 1 << 16;  
    asm("dsb");
}   


void adcetc0_isr() {
    //digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    ADC_ETC_DONE0_1_IRQ |= 1;   // clear
    const uint16_t val0 = ADC_ETC_TRIG1_RESULT_1_0 & 4095;
    asm("dsb");
  }
  
  void adcetc1_isr() {
    //digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    ADC_ETC_DONE0_1_IRQ |= 1 << 16;   // clear
    const uint16_t val1 = (ADC_ETC_TRIG1_RESULT_1_0 >> 16) & 4095;
    asm("dsb");
  }

void Bldc::configureADCs(){
    // Use XBAR to connect PWM4.2 to ADC ETC 0
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
    xbar_connect(XBARA1_IN_FLEXPWM4_PWM2_OUT_TRIG1, XBARA1_OUT_ADC_ETC_TRIG10);

    // Configure ADC2 oversample and 12 bit conversion
    IMXRT_ADC2.CFG = ADC_CFG_AVGS(0b01) | ADC_CFG_MODE(0b10) | ADC_CFG_ADTRG;
    // Continuous conversion and enable averaging by hardware
    IMXRT_ADC2.GC = ADC_GC_AVGE;  // ADC_GC_ADCO | 

    // Register Interrupt 
    // attachInterruptVector(IRQ_ADC2, ADC2_CompletedConversionCallback);
    // NVIC_ENABLE_IRQ(IRQ_ADC2);

    // Soft Reset the ADC_ETC module
    IMXRT_ADC_ETC.CTRL = ADC_ETC_CTRL_SOFTRST;  
    IMXRT_ADC_ETC.CTRL &= ~ADC_ETC_CTRL_SOFTRST; 
    delay(5);
    // Enable External Signal Controller 1
    //ADC_ETC_TRIG0_CTRL = 0x100;
    IMXRT_ADC_ETC.CTRL = ADC_ETC_CTRL_TRIG_ENABLE((1 << 1));
    //ADC_ETC_TRIG0_CHAIN_1_0 = 0x50283017;

    // IMXRT_ADC_ETC.TRIG[1].CHAIN_1_0 =
    IMXRT_ADC_ETC.TRIG[1].CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE0(1)  /* Enable interrupt for first conversion */
    | ADC_ETC_TRIG_CHAIN_HWTS0(1)  /* Select Hardware Trigger Source 0 */
    | ADC_ETC_TRIG_CHAIN_CSEL0(1)  /* First conversion: ADC2 channel 1 */
    
    | ADC_ETC_TRIG_CHAIN_IE1(1)  /* Enable interrupt for second conversion */
    | ADC_ETC_TRIG_CHAIN_CSEL1(3)  /* Second conversion: ADC2 channel 3 */
    ;
    
    // START ADC's
    //ADC2_HC0 = 16;
    IMXRT_ADC2.HC0 = ADC_HC_ADCH(1) | ADC_HC_AIEN;
    //IMXRT_ADC2.HC1 = ADC_HC_ADCH(3) | ADC_HC_AIEN;
    //IMXRT_ADC2.HC2 = ADC_HC_ADCH(4) | ADC_HC_AIEN;

    // Link external trigger interrupt to callback
    // attachInterruptVector(IRQ_ADC_ETC0, ADC2_CompletedConversionCallback);
    // NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
    // attachInterruptVector(IRQ_ADC_ETC1, ADC2_CompletedConversionCallback);
    // NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);

    attachInterruptVector(IRQ_ADC_ETC0, adcetc0_isr);
    NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
    attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
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