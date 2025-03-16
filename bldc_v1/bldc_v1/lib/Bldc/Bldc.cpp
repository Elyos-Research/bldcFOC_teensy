#include "Bldc.h"

static Bldc* instance = nullptr;

void PWM2_CompletedCallback(){
    if (IMXRT_FLEXPWM2.SM[0].STS & 0x1) {  // HF (Half Period Flag)
        IMXRT_FLEXPWM2.SM[0].STS = 0x1;  // Clear HF flag
        triggerADC();
    }
    
    if (IMXRT_FLEXPWM2.SM[0].STS & 0x2) {
        instance->newCycle = true;
        IMXRT_FLEXPWM2.SM[0].STS = 0x3; // Clear FF flag
        IMXRT_FLEXPWM2.SM[0].INTEN = 0x3; // Re-enable half-period and full period interrupt (FF)
    }

}

void ADC1_CompletedConversionCallback(){
    if (instance) {  // Check if the ADC conversion is complete
        if (ADC1_HS & ADC_HS_COCO0) {
            instance->newThrottleVal = true;
            uint32_t result = ADC1_R0; // Read the ADC result
            instance->throttleRawVal = result; // Update throttleVal
        }
    }
}


Bldc::Bldc() : controlType(ControlType::Trap), hallState(0), throttleRawVal(0), throttleNormVal(0), newCycle(false) {
    instance = this; // Set the static instance pointer
    // Initialize phase configurations
    phaseA = {0, 0, kGateAH, Phase::Mode::X, 1};
    phaseB = {0, 0, kGateBH, Phase::Mode::X, 2};
    phaseC = {0, 0, kGateCH, Phase::Mode::X, 3};
}

Bldc::~Bldc() {}

void Bldc::driverInit() {
    // Initialize Hall sensor pins
    pinMode(kHallAPin, INPUT);
    pinMode(kHallBPin, INPUT);
    pinMode(kHallCPin, INPUT);

    // Initialize debug pins
    pinMode(kBuiltInLedPin, OUTPUT);
    pinMode(kIrqFlagPin, OUTPUT);

    // Configure hardware modules
    configurePWM();
    configureADC();

#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    // Trigger ADC conversion
    triggerADC();
}

void Bldc::run() {
    setPhaseDuty(0,0,0);
}

void Bldc::configurePWM() {
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
    IMXRT_FLEXPWM2.SM[0].STS = 0x3;
    IMXRT_FLEXPWM2.SM[0].INTEN = 0x3;
    
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
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, kPwmFrequency);

    // Write zeros to everything
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, 0, phaseA.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, 0, phaseA.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, 0, phaseB.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, 0, phaseB.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, 0, phaseC.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, 0, phaseC.mode);

    // Configure signal propagation to pads
    #define portConfigRegister(pin)  ((digital_pin_to_info_PGM[(pin)].mux))
    *(portConfigRegister(kGateAH)) =  1;  // GATE AH
    *(portConfigRegister(kGateAL)) =  1;  // GATE AL
    *(portConfigRegister(kGateBH)) =  2;  // GATE BH
    *(portConfigRegister(kGateBL)) =  2;  // GATE BL
    *(portConfigRegister(kGateCH)) =  6;  // GATE CH
    *(portConfigRegister(kGateCL)) =  6;  // GATE CL
    
    //Enable outputs in A and B 
    IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 3) | FLEXPWM_OUTEN_PWMB_EN(1 << 3) 
                         | FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2)
                         | FLEXPWM_OUTEN_PWMA_EN(1) | FLEXPWM_OUTEN_PWMB_EN(1);

    delay(1);
}

void Bldc::configureADC() {
    uint32_t mode = ADC_CFG_ADICLK(0b00) | ADC_CFG_MODE(0b10) | ADC_CFG_ADLSMP | ADC_CFG_ADIV(0b00) | ADC_CFG_ADSTS(0b11) | ADC_CFG_AVGS(0b10) | ADC_CFG_OVWREN;
    uint32_t avg = (ADC_GC_AVGE & (~ADC_GC_ADCO)) | ADC_GC_CAL;

    // Configure ADC1
    ADC1_CFG = mode;
    ADC1_GC = avg;
    while (ADC1_GC & ADC_GC_CAL) {
        yield();  // Wait until calibration is complete
    }

    // Configure ADC2
    ADC2_CFG = mode;
    ADC2_GC = avg;
    while (ADC2_GC & ADC_GC_CAL) {
        yield();  // Wait until calibration is complete
    }

    // Register ADC interrupt
    attachInterruptVector(IRQ_ADC1, ADC1_CompletedConversionCallback);
    NVIC_ENABLE_IRQ(IRQ_ADC1);
}

void Bldc::readHalls(uint8_t &hall) {
    uint8_t hallCountsC = 0, hallCountsB = 0, hallCountsA = 0;
    
    for (uint8_t i = 0; i < kHallOverSample; i++) {
        hallCountsC += digitalRead(kHallCPin);
        hallCountsB += digitalRead(kHallBPin);
        hallCountsA += digitalRead(kHallAPin);
    }
    
    hall = ((hallCountsC > kHallOverSample / 2) << 2) |
           ((hallCountsB > kHallOverSample / 2) << 1) |
           ((hallCountsA > kHallOverSample / 2) << 0);
}


void Bldc::nextStep(uint8_t &currentHallState) {
    /** Override this method */
    #ifdef SERIAL_DEBUG
        Serial.println("Override nextStep method");
    #endif
}

void Bldc::setGatePWM(Phase phase){
  if (phase.phaseID == 1)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, phase.pwmVal, phase.mode);
    #ifdef SERIAL_DEBUG
        Serial.print("\tA> ");
        Serial.print(phase.pwmVal);
    #endif
  }
  if (phase.phaseID == 2)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, phase.pwmVal, phase.mode);
    #ifdef SERIAL_DEBUG
        Serial.print(" B> ");
        Serial.print(phase.pwmVal);
    #endif
  }
  if (phase.phaseID == 3)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, phase.pwmVal, phase.mode);
    #ifdef SERIAL_DEBUG
        Serial.print(" C> ");
        Serial.println(phase.pwmVal);
    #endif
  }
}

void Bldc::setPhaseDuty(int16_t phaseApwm, int16_t phaseBpwm, int16_t phaseCpwm){
    if (phaseApwm > 1){ 
        phaseA.pwmVal = 4096 - phaseApwm;
        phaseA.mode = Phase::Mode::Complementary;
        setGatePWM(phaseA); 
    } else if (phaseApwm == 0){ 
        phaseA.pwmVal = -1;
        phaseA.mode = Phase::Mode::X;
        setGatePWM(phaseA); 
    } else { 
        phaseA.pwmVal = -1;
        phaseA.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseA); 
    }
    
    if (phaseBpwm > 1){ 
        phaseB.pwmVal =  4096 - phaseBpwm;
        phaseB.mode = Phase::Mode::Complementary;
        setGatePWM(phaseB); 
    } else if (phaseBpwm == 0){ 
        phaseB.pwmVal = -1;
        phaseB.mode = Phase::Mode::X;
        setGatePWM(phaseB); 
    } else { 
        phaseB.pwmVal = -1;
        phaseB.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseB); 
    }

    if (phaseCpwm > 1){ 
        phaseC.pwmVal = 4096 - phaseCpwm;
        phaseC.mode = Phase::Mode::Complementary;
        setGatePWM(phaseC); 
    } else if (phaseCpwm == 0){ 
        phaseC.pwmVal = 0;
        phaseC.mode = Phase::Mode::X;
        setGatePWM(phaseC); 
    } else { 
        phaseC.pwmVal = -1;
        phaseC.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseC); 
    }
}

void Bldc::normThrottle(uint16_t &throttle) {
    uint16_t throttleRaw = analogRead(kThrottlePin);
    throttleRaw = constrain(throttleRaw, kThrottleLow, kThrottleHigh);
    throttle = map(throttleRaw, kThrottleLow, kThrottleHigh, 0, kThrottleResolution);
}

void Bldc::readCurrents(int16_t &currentA, int16_t &currentB, int16_t &currentC) {
    currentA = analogRead(kCurrentSenseA);
    currentB = analogRead(kCurrentSenseB);
    currentC = analogRead(kCurrentSenseC);
}

void Bldc::setPwmFrequency(IMXRT_FLEXPWM_t *p, uint8_t submodule, uint8_t channel, float frequency) {
    uint16_t mask = 1 << submodule;
    uint32_t newdiv = static_cast<uint32_t>((F_BUS_ACTUAL / frequency) + 0.5f);
    uint8_t prescale = 0;

    while (newdiv > 65535 && prescale < 7) {
        newdiv >>= 1;
        prescale++;
    }

    newdiv = constrain(newdiv, 2, 65535);

    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    p->SM[submodule].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);
    p->SM[submodule].INIT = -(newdiv / 2);
    p->SM[submodule].VAL1 = (newdiv / 2);
    p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}

void Bldc::writePwmValue(IMXRT_FLEXPWM_t *p, uint8_t submodule, uint8_t channel, int16_t value, Phase::Mode mode) {
    if (mode == Phase::Mode::Complementary || mode == Phase::Mode::ZeroComplement){
        IMXRT_FLEXPWM2.SM[submodule & 0xF].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    } else{
        IMXRT_FLEXPWM2.SM[submodule & 0xF].OCTRL |= FLEXPWM_SMOCTRL_POLB; 
    }
    
    uint16_t mask = 1 << submodule;
    uint32_t modulo = p->SM[submodule].VAL1 * 2;
    uint32_t cval = ((uint32_t)value * (modulo + 1)) >> 12;

    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    switch (channel) {
        case 0:  // X
            p->SM[submodule].VAL0 = modulo - cval;
            p->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
            break;
        case 1:  // A
            if (mode == Phase::Mode::Complementary) {
                p->SM[submodule].VAL2 = -(cval / 2);
                p->SM[submodule].VAL3 = (cval / 2)+1;
            } else if (mode == Phase::Mode::ZeroComplement) {
                p->SM[submodule].VAL2 = p->SM[submodule].INIT;
                p->SM[submodule].VAL3 = p->SM[submodule].VAL1;
            } else {
                p->SM[submodule].VAL2 = p->SM[submodule].INIT;
                p->SM[submodule].VAL3 = p->SM[submodule].INIT;
            }
                p->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
            
            break;
        case 2:  // B
            if (mode == Phase::Mode::Complementary) {
                p->SM[submodule].VAL4 = -(cval / 2);
                p->SM[submodule].VAL5 = (cval / 2)+1;
            } else if (mode == Phase::Mode::ZeroComplement) {
                p->SM[submodule].VAL4 = p->SM[submodule].INIT;
                p->SM[submodule].VAL5 = p->SM[submodule].VAL1;
            } else {
                p->SM[submodule].VAL4 = p->SM[submodule].INIT;
                p->SM[submodule].VAL5 = p->SM[submodule].INIT;
            }
                p->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
            break;
    }
    p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}


void triggerADC() {
    uint8_t ch = 0x01 | (1 << 7); // Set channel and enable interrupt
    ADC1_HC0 = ch;
}

void toggleLed(){
  digitalWrite(kBuiltInLedPin, !digitalRead(kBuiltInLedPin));
}   

void toggleFlag(){
  digitalWrite(kIrqFlagPin, !digitalRead(kIrqFlagPin));
}














// int readADCxd(u_int16_t gpio){
//   uint8_t ch;
//   if(gpio == kThrottlePin){
//     ch = 0x01;
//     ch |= 1 << 7;
//     ADC1_HC0 = ch;
//     while (!(ADC1_HS & ADC_HS_COCO0)) {
//       yield(); 
//     }
//     return ADC1_R0;
//   }

//   if(gpio == kCurrentSenseA){
//     ch = 0x03; 
//     ADC2_HC0 = ch;
//     while (!(ADC2_HS & ADC_HS_COCO0)) {
//       yield(); 
//     }
//     return ADC2_R0;
//   }

//   if(gpio == kCurrentSenseB){
//     ch = 0x04;
//     ADC2_HC0 = ch;
//     while (!(ADC2_HS & ADC_HS_COCO0)) {
//       yield(); 
//     }
//     return ADC2_R0;
//   }

//   if(gpio == kCurrentSenseC){
//     ch = 0x01;
//     ADC2_HC0 = ch;
//     while (!(ADC2_HS & ADC_HS_COCO0)) {
//       yield();
//     }
//     return ADC2_R0;
//   }
//   return 0;
// }
















































