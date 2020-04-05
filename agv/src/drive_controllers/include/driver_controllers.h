#pragma once
#include <utility>
#include <vector>

namespace avc {
class MotorController {
  public:
    MotorController() = default;
    MotorController(size_t vcc, size_t a1, size_t a2, size_t b1, size_t b2);
    void turnOn();
    void turnOff();
    void forward(size_t i);
    void backward(size_t i);
    void stop(size_t i);

    size_t size() const { return motors_; }

  private:
    int vcc_;
    std::vector<std::pair<size_t, size_t>> motorPins_;
    size_t motors_;
};
}