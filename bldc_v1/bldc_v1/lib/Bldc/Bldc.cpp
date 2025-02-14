// #include "Bldc.h"

// Bldc::Bldc()
// {
//     throttle_raw = 0;  
//     hall_state = 0;  
//     blink_aux = 0;
//     throttle_aux = 0;
//     print_aux = 0; 
// }

// Bldc::~Bldc(){}

// void Bldc::driverInit(){
//   // GPIO Initialization
//   pinMode(BUILTIN_LED_PIN, OUTPUT);

//   // ADC Config 
//   analogReadResolution(ANALOG_RESOLUTION);
//   analogReadAveraging(ANALOG_AVERAGING);

//   // Hall Sensor Inputs
//   pinMode(HALL_A_PIN, INPUT_PULLUP);
//   pinMode(HALL_B_PIN, INPUT_PULLUP);
//   pinMode(HALL_C_PIN, INPUT_PULLUP);

//   //Init pwm on gate drivers pins

//   // Debugging Serial Output
//   Serial.begin(9600);
// }

// void Bldc::identifyHalls(uint8_t &current_hall_state){
//     uint8_t aux = ((digitalRead(HALL_A_PIN) << 2) | (digitalRead(HALL_B_PIN) << 1) | (digitalRead(HALL_C_PIN)));
//     // Discard invalid positions
//     if(aux == 0 || aux == 7){
//     return;
//     }
//     current_hall_state = aux;
// }

// void Bldc::setPhaseDuty(uint16_t &h_a, uint16_t &l_a, uint16_t &h_b, uint16_t &l_b, uint16_t &h_c, uint16_t &l_c){
//   // Phase A
//   setGatePWM(GATE_AH, h_a);
//   setGatePWM(GATE_AL, l_a);
//   // Phase B
//   setGatePWM(GATE_BH, h_b);
//   setGatePWM(GATE_BL, l_b);
//   // Phase C
//   setGatePWM(GATE_CH, h_c);
//   setGatePWM(GATE_CL, l_c);
// }

// void Bldc::readThrottle(uint16_t &throttle){
//     /*Read adc*/
// }

// void Bldc::setGatePWM(int gate, uint16_t pwm){
//     /*Set pwm to a given port if its a low gate set oposite of high and a margin to avoid shortcut in the half bridge*/
// }


// void Bldc::runTrapezoidAlgo(void){
//     /*Run algorithm for trapezoid algo*/
// }