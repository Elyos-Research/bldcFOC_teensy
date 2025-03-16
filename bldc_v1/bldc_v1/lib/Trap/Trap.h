#ifndef TRAP_H
#define TRAP_H

#include "Bldc.h"

class Trap : public Bldc
{
public:
    Trap();
    ~Trap();
    void run() override;
private:
    void writeTrap(uint8_t &halls, uint16_t duty);
    void nextStep(uint8_t &currentHallState) override;
};

#endif //TRAP_H