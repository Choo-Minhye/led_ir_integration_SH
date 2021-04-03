#ifndef _INFRARED_H_
#define _INFRARED_H_

#include "Arduino.h"

class InfraRed
{
private:
  const int read_pin_;
  const int light_pin_;

public:
  InfraRed() = delete;
  InfraRed(const int, const int);
  InfraRed &lightControl(const bool);
  bool isOccupied() const;
};

InfraRed::InfraRed(const int read_pin, const int light_pin) : read_pin_(read_pin), light_pin_(light_pin)
{
  pinMode(read_pin_, INPUT);
  pinMode(light_pin_, OUTPUT);
}

InfraRed &InfraRed::lightControl(const bool val)
{
  digitalWrite(light_pin_, val);
  return *this;
}

bool InfraRed::isOccupied() const
{
  return !digitalRead(read_pin_);
}

#endif
