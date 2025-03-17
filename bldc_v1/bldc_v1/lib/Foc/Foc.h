#ifndef FOC_H
#define FOC_H

#include "Bldc.h"

class Foc : public Bldc {
public:
    Foc();
    ~Foc();
    void run() override;
    
private:
    // FOC control variables
    float id_ref;   // d-axis current reference (often set to zero)
    float iq_ref;   // q-axis current reference (torque-producing)
    float id;       // measured d-axis current
    float iq;       // measured q-axis current
    float angle;    // rotor electrical angle (an estimation based on sensors)
    
    // PI controller gains for d- and q-axis loops
    float kp_d, ki_d;
    float kp_q, ki_q;
    float integral_d, integral_q;
    
    // Transforms and control functions
    void clarkeTransform(float ia, float ib, float ic, float &i_alpha, float &i_beta);
    void parkTransform(float i_alpha, float i_beta, float angle, float &id, float &iq);
    void inverseParkTransform(float vd, float vq, float angle, float &v_alpha, float &v_beta);
    void currentControl(float id_measured, float iq_measured, float id_ref, float iq_ref, float &vd, float &vq);
};

#endif // FOC_H
