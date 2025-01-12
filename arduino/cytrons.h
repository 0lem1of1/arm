#ifndef CYTRONS_H
#define CYTRONS_H

#include "Arduino.h"

class MDD10A {
  public:
    MDD10A(int pwm, int dir, int enca, int encb);
    ~MDD10A();

    void run(int pwr);
    void stop();
    void attachEncInterrupt();
    int getEncoderCount() const;

  private:
    int _pwm_pin;
    int _dir_pin;
    int _enca;
    int _encb;

    volatile int _encoderCount; // Shared variable for ISR
    static void encoderInterruptA(); // Static ISR for A

    static MDD10A* _instance; // Pointer to current instance for ISRs
};

#endif