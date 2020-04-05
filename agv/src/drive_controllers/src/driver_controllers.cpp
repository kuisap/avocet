#include "driver_controllers.h"
#include <wiringPi.h>

namespace avc
{
  MotorController::MotorController(size_t vcc, size_t a1, size_t a2, size_t b1, size_t b2) : motors_(2) {
    motorPins_.push_back({a1, a2});
    motorPins_.push_back({b1, b2});
    vcc_ = vcc;
    wiringPiSetup();
    pinMode(vcc_, OUTPUT);

    for (const auto& [i1, i2] : motorPins_) {
      pinMode(i1, OUTPUT);
      pinMode(i2, OUTPUT);
    }
  }

  void MotorController::turnOn() {
    digitalWrite(vcc_, HIGH);
    for (const auto& [i1, i2] : motorPins_) {
      digitalWrite(i1, LOW);
      digitalWrite(i2, LOW);
    }
  }

  void MotorController::turnOff() {
    for (const auto& [i1, i2] : motorPins_) {
      digitalWrite(i1, LOW);
      digitalWrite(i2, LOW);
    }
    digitalWrite(vcc_, LOW);
  }

  void MotorController::forward(size_t i){
    const auto& [i1, i2] = motorPins_[i];
    digitalWrite(i1, HIGH);
    digitalWrite(i2, LOW);
  }


  void MotorController::backward(size_t i){
    const auto& [i1, i2] = motorPins_[i];
    digitalWrite(i1, LOW);
    digitalWrite(i2, HIGH);
  }

  void MotorController::stop(size_t i){
    const auto& [i1, i2] = motorPins_[i];
    digitalWrite(i1, LOW);
    digitalWrite(i2, LOW);
  }

}
