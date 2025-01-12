#include "cytrons.h"
#include <math.h>

MDD10A* MDD10A::_instance = nullptr;

MDD10A::MDD10A(int pwm, int dir, int enca, int encb)
  : _pwm_pin(pwm), _dir_pin(dir), _enca(enca), _encb(encb), _encoderCount(0) {
  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_enca, INPUT_PULLUP);
  pinMode(_encb, INPUT_PULLUP);

  if (enca == 0 && encb == 0) {
    _instance = this;
  }
}

MDD10A::~MDD10A() {
  // PASS
}

void MDD10A::run(int pwr) {
  int dir = (pwr > 0) ? 1 : 0; // Setting to HIGH when pwr is +ve, setting to LOW when pwr is 0 / -ve
  int u = min(abs(pwr), 255);

  analogWrite(_pwm_pin, u);
  digitalWrite(_dir_pin, dir);
}

void MDD10A::stop() {
  analogWrite(_pwm_pin, 0); // Setting PWM to 0 to stop the motor
  digitalWrite(_dir_pin, LOW); // Setting DIR to LOW to not waste any energy after stopping
}

void MDD10A::encoderInterruptA() {
  if (_instance) {
    if (digitalRead(_instance->_enca) == digitalRead(_instance->_encb)) {
      _instance->_encoderCount++;
    } else {
      _instance->_encoderCount--;
    }
  }
}

void MDD10A::attachEncInterrupt() {
  attachInterrupt(digitalPinToInterrupt(_enca), encoderInterruptA, CHANGE);
}

int MDD10A::getEncoderCount() const {
  return _encoderCount;
}