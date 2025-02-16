#include <Arduino.h>
#include "Bldc.h"

Bldc bldc;
bool irq_flag_state;

void setup(){
  bldc.driverInit();
  irq_flag_state = false;
}

void loop(){
  bldc.run();
  Serial.println(".+"); //< just to see if doest break

  delay(100);
}