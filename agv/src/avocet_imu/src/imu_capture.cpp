#include <avocet_imu/imu_capture.h>

#include <bitset>
#include <iostream>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <chrono>

#define INFO(x) std::cout << x << std::endl;

namespace avc {

namespace {

constexpr float gravity = 9.80665F;

enum class AccResolution : int {
  G2 = 0b0011,
  G4 = 0b0101,
  G8 = 0b1000,
  G16 = 0b1100
};

enum class GyroResolution : int {
  A2000 = 0b000,
  A1000 = 0b001,
  A500 = 0b010,
  A250 = 0b011,
  A125 = 0b100
};

std::map<AccResolution, float> accResolution = {
    {AccResolution::G2, 0.98F * 1e-3 * gravity},
    {AccResolution::G4, 1.95F * 1e-3 * gravity},
    {AccResolution::G8, 3.91F * 1e-3 * gravity},
    {AccResolution::G16, 7.81F * 1e-3 * gravity},
};

std::map<GyroResolution, float> gyroResolution = {
    {GyroResolution::A2000, 61.F * 1e-3}, {GyroResolution::A1000, 30.5F * 1e-3},
    {GyroResolution::A500, 15.3F * 1e-3}, {GyroResolution::A250, 7.6F * 1e-3},
    {GyroResolution::A125, 3.8F * 1e-3},
};

struct Addresses {
  struct AccSubAddress {
    const int chipId = 0x00;
    const int accXLSB = 0x02;
    const int accXMSB = 0x03;
    const int accYLSB = 0x04;
    const int accYMSB = 0x05;
    const int accZLSB = 0x06;
    const int accZMSB = 0x07;
    const int bandWidth = 0x10;
    const int reset = 0x14;
  };

  struct GyroSubAddress {
    const int chipId = 0x00;
    const int gyroXLSB = 0x02;
    const int gyroXMSB = 0x03;
    const int gyroYLSB = 0x04;
    const int gyroYMSB = 0x05;
    const int gyroZLSB = 0x06;
    const int gyroZMSB = 0x07;
  };

  struct GeoSubAddress {
    const int chipId = 0x40;
  };

  const int accAddress = 0x19;
  const AccSubAddress accSubAddress;
  const int gyroAddress = 0x69;
  const GyroSubAddress gyroSubAddress;
  const int geoAddress = 0x13;
  const GeoSubAddress geoSubAddress;
};

struct ValidationData {
  struct AccData {
    const int chipSet = 0b11111010;
  };
  struct GyroData {
    const int chipSet = 0b00001111;
  };
  struct GeoData {
    const int chipSet = 0b00110010;
  };

  const AccData accData;
  const GyroData gyroData;
  const GeoData geoData;
};

Addresses addresses;
ValidationData validationData;

std::tuple<bool, float> parseAcc(int accLSB, int accMSB, float resolution) {
  using Bit16 = std::bitset<16>;
  const Bit16 lsbBit(accLSB);
  const Bit16 msbBit(accMSB);
  const auto isNewData = (lsbBit & Bit16(0x0001))[0];
  std::bitset<16> accB(0);
  accB |= (msbBit << 4) | ((lsbBit & Bit16(0x00F0)) >> 4);
  if (accB[11]) {
    accB[15] = accB[14] = accB[13] = accB[12] = true;
  }
  return {isNewData, static_cast<short>(accB.to_ulong()) * resolution};
}

float parseGyro(int gyroLSB, int gyroMSB, float resolution) {
  short value = (gyroMSB << 8) | gyroLSB;
  return value * resolution;
}

} // namespace

class IMUCapture::IMUCaptureImpl {
public:
  IMUCaptureImpl() : isEnable_(false), accFd_(-1), gyroFd_(-1), geoFd_(-1) {

    accResolution_ = AccResolution::G2;
    gyroResolution_ = GyroResolution::A2000;

    accFd_ = wiringPiI2CSetup(addresses.accAddress);
    if (accFd_ < 0) {
      throw std::runtime_error("Acceralater is not available.");
    }

    gyroFd_ = wiringPiI2CSetup(addresses.gyroAddress);
    if (gyroFd_ < 0) {
      throw std::runtime_error("Gyro is not available.");
    }

    geoFd_ = wiringPiI2CSetup(addresses.geoAddress);
    if (geoFd_ < 0) {
      throw std::runtime_error("Geo is not available.");
    }

    isEnable_ = validate();
  }

  bool validate() const {
    {
      const auto accChipId =
          wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.chipId);
      if (accChipId != validationData.accData.chipSet) {
        throw std::runtime_error("Acceralater chip set is invalid.");
      }
    }

    {
      const auto gyroChipId =
          wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.chipId);
      if (gyroChipId != validationData.gyroData.chipSet) {
        throw std::runtime_error("Gyro chip set is invalid.");
      }
    }

    return true;
  }

  IMUData sample() const {

    IMUData data;
    {
      data.accIsNew = true;
      {
        const auto accXLSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accXLSB);
        const auto accXMSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accXMSB);
        const auto [isNewData, accX] =
            parseAcc(accXLSB, accXMSB, accResolution[accResolution_]);
        data.accX = accX;
        data.accIsNew &= isNewData;
      }

      {
        const auto accYLSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accYLSB);
        const auto accYMSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accYMSB);
        const auto [isNewData, accY] =
            parseAcc(accYLSB, accYMSB, accResolution[accResolution_]);
        data.accY = accY;
        data.accIsNew &= isNewData;
      }

      {
        const auto accZLSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accZLSB);
        const auto accZMSB =
            wiringPiI2CReadReg8(accFd_, addresses.accSubAddress.accZMSB);
        const auto [isNewData, accZ] =
            parseAcc(accZLSB, accZMSB, accResolution[accResolution_]);
        data.accZ = accZ;
        data.accIsNew &= isNewData;
      }
    }

    {
      {
        const auto gyroXLSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroXLSB);
        const auto gyroXMSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroXMSB);
        data.gyroX =
            std::move(parseGyro(gyroXLSB, gyroXMSB, gyroResolution[gyroResolution_]));
      }

      {
        const auto gyroYLSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroYLSB);
        const auto gyroYMSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroYMSB);
        data.gyroY =
            std::move(parseGyro(gyroYLSB, gyroYMSB, gyroResolution[gyroResolution_]));
      }

      {
        const auto gyroZLSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroZLSB);
        const auto gyroZMSB =
            wiringPiI2CReadReg8(gyroFd_, addresses.gyroSubAddress.gyroZMSB);
        data.gyroZ =
            std::move(parseGyro(gyroZLSB, gyroZMSB, gyroResolution[gyroResolution_]));
      }
    }

    return data;
  }

  bool isEnable() const { return isEnable_; }

private:
  bool isEnable_;
  int accFd_, gyroFd_, geoFd_;
  AccResolution accResolution_;
  GyroResolution gyroResolution_;
};

IMUCapture::IMUCapture()
    : impl_(std::move(std::make_unique<IMUCaptureImpl>())) {}

IMUCapture::~IMUCapture() {}

bool IMUCapture::isEnable() const { return impl_->isEnable(); }

IMUData IMUCapture::sample() const { return impl_->sample(); }
} // namespace avc
