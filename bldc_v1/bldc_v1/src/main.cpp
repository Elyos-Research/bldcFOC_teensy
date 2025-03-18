#include <Arduino.h>
#include "Bldc.h"
#include "Trap.h"
#include "Foc.h"

#ifdef FOC_CONTROL
  Foc bldc;
#else
  Trap bldc;
#endif

bool irq_flag_state;

void setup(){
  bldc.driverInit();
  irq_flag_state = false;
}

void loop(){
  bldc.run();
}