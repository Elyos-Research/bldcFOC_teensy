// #include "Foc.h"
// #include <Arduino.h>
// #include <math.h>

// #define TWO_PI 6.28318530718f

// Foc::Foc() : id_ref(0.0f), iq_ref(0.0f), id(0.0f), iq(0.0f), angle(0.0f),
//              kp_d(0.1f), ki_d(0.01f), kp_q(0.1f), ki_q(0.01f),
//              integral_d(0.0f), integral_q(0.0f)
// {
//     // Set control type to FOC so that any base class behavior can differ if needed
//     controlType = ControlType::Foc;
// }

// Foc::~Foc() {}

// void Foc::run() {
//     // Update FOC control once per PWM cycle
//     if (newCycle) {
//         // Read hall sensors to get a crude estimate of rotor position.
//         // In a full FOC implementation, you might use an encoder or observer.
//         uint8_t hall = 0;
//         readHalls(hall);
//         // For simplicity, map the hall state (0 to 7) into an electrical angle.
//         // Adjust this mapping as needed.
//         angle = (float)hall / 6.0f * TWO_PI;

//         // Read throttle and update iq reference.
//         // Here we assume throttleNormVal (0–4096) maps directly to a voltage reference.
//         uint16_t throttle = throttleNormVal;
//         iq_ref = (float)throttle / 4096.0f;  // Scale throttle to [0,1] range
//         // In many FOC strategies id_ref is set to zero (maximizing torque per amp).
//         id_ref = 0.0f;

//         // Read the phase currents.
//         // Here we assume currentA, currentB and currentC are updated by your ADC callbacks.
//         float ia = (float)currentA;  // Conversion factors may be needed in a real system.
//         float ib = (float)currentB;
//         float ic = (float)currentC;
        
//         // Perform Clarke transform to get i_alpha and i_beta.
//         float i_alpha, i_beta;
//         clarkeTransform(ia, ib, ic, i_alpha, i_beta);
        
//         // Perform Park transform to get d– and q–axis currents.
//         parkTransform(i_alpha, i_beta, angle, id, iq);

//         // Run PI current controllers for both axes.
//         float vd, vq;
//         currentControl(id, iq, id_ref, iq_ref, vd, vq);

//         // Convert the voltage references back to the stationary frame.
//         float v_alpha, v_beta;
//         inverseParkTransform(vd, vq, angle, v_alpha, v_beta);

//         // Convert these voltage references into PWM duty cycles.
//         // This example uses a placeholder mapping.
//         int16_t dutyA = (int16_t)((v_alpha + 1.0f) * 2048);  // Adjust scaling as required
//         int16_t dutyB = (int16_t)((v_beta  + 1.0f) * 2048);
//         // For phase C, one common method is to use the zero–sequence component.
//         int16_t dutyC = (int16_t)((-v_alpha - v_beta + 1.0f) * 2048);
        
//         // Update the PWM outputs.
//         setPhaseDuty(dutyA, dutyB, dutyC);
        
//         newCycle = false;
//     }
    
//     // Update throttle if a new throttle value has been acquired.
//     if (newThrottleVal) {
//         uint16_t throttle;
//         throttle = (uint16_t)throttleRawVal;
//         normThrottle(throttle);
//         throttleNormVal = throttle;
//         newThrottleVal = false;
//     }
// }

// // --- Transform and Control Functions ---

// // Clarke Transform: convert three-phase currents to two orthogonal components.
// // Assumes balanced currents where ia + ib + ic = 0.
// void Foc::clarkeTransform(float ia, float ib, float ic, float &i_alpha, float &i_beta) {
//     // One common form:
//     i_alpha = ia;
//     i_beta = (1.0f / sqrt(3.0f)) * (ia + 2.0f * ib);
// }

// // Park Transform: rotate the αβ frame into the dq frame using the rotor angle.
// void Foc::parkTransform(float i_alpha, float i_beta, float angle, float &id, float &iq) {
//     id = i_alpha * cos(angle) + i_beta * sin(angle);
//     iq = -i_alpha * sin(angle) + i_beta * cos(angle);
// }

// // Inverse Park Transform: convert dq voltage commands back into the αβ frame.
// void Foc::inverseParkTransform(float vd, float vq, float angle, float &v_alpha, float &v_beta) {
//     v_alpha = vd * cos(angle) - vq * sin(angle);
//     v_beta  = vd * sin(angle) + vq * cos(angle);
// }

// // Simple PI current control for d- and q–axis loops.
// void Foc::currentControl(float id_measured, float iq_measured, float id_ref, float iq_ref, float &vd, float &vq) {
//     float error_d = id_ref - id_measured;
//     float error_q = iq_ref - iq_measured;
    
//     // Integrate the errors.
//     integral_d += error_d;
//     integral_q += error_q;
    
//     // PI control output.
//     vd = kp_d * error_d + ki_d * integral_d;
//     vq = kp_q * error_q + ki_q * integral_q;
// }
