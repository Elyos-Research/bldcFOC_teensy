#ifndef PINOUT_H
#define PINOUT_H

#define SERIAL_DEBUG
//#define BACKWARDS

#include <cstdint>

// HALL Sensors
constexpr uint8_t kHallAPin{12}; // Pin for Hall sensor A
constexpr uint8_t kHallBPin{11}; // Pin for Hall sensor B
constexpr uint8_t kHallCPin{10}; // Pin for Hall sensor C

// Hall Parameters
constexpr uint8_t kHallOverSample{8};

// DRIVER GATES (PWM Outputs)
constexpr uint8_t kGateAH{4};  // High-side gate for phase A
constexpr uint8_t kGateAL{33}; // Low-side gate for phase A
constexpr uint8_t kGateBH{6};  // High-side gate for phase B
constexpr uint8_t kGateBL{9};  // Low-side gate for phase B
constexpr uint8_t kGateCH{36}; // High-side gate for phase C
constexpr uint8_t kGateCL{37}; // Low-side gate for phase C

// DRIVERS GATES PARAMS
constexpr uint32_t kPwmFrequency{30000}; // 30 kHz PWM frequency

// Other Pins
constexpr uint8_t kBuiltInLedPin{13}; // Built-in LED pin
constexpr uint8_t kIrqFlagPin{30}; // Irq flag pin

// ADC Inputs
constexpr uint8_t kThrottlePin{A10};    // Throttle input pin
constexpr uint8_t kCurrentSenseA{A12}; // Current sense for phase A
constexpr uint8_t kCurrentSenseB{A13}; // Current sense for phase B
constexpr uint8_t kCurrentSenseC{A14}; // Current sense for phase C

// ADC Resolution & Sampling
constexpr uint8_t kAnalogResolution{12}; // 12-bit ADC
constexpr uint8_t kAnalogAveraging{8};   // 8x oversampling

// Throttle Configuration
constexpr uint16_t kThrottleLow{10};     // Minimum throttle value
constexpr uint16_t kThrottleHigh{4094};  // Maximum throttle value
constexpr uint16_t kThrottleResolution{4095}; // Throttle resolution (12-bit)


//////////////////////////////////////////////////
//////////////// DEBUG MACROS ///////////////////
//////////////////////////////////////////////////

#ifdef SERIAL_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif // PINOUT_H













