#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <memory>

namespace avc {

struct IMUData {
  float accX;
  float accY;
  float accZ;
  bool accIsNew;
  float gyroX;
  float gyroY;
  float gyroZ;
  bool gyroIsNew;
  float geoX;
  float geoY;
  float geoZ;
  bool geoIsNew;
};

class IMUCapture {
public:
  IMUCapture();
  ~IMUCapture();
  IMUData sample() const;
  bool isEnable() const;

private:
  class IMUCaptureImpl;
  std::unique_ptr<IMUCaptureImpl> impl_;
};
} // namespace avc
