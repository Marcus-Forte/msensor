#pragma once

#include "interface/IImu.hh"
#include <stdint.h>


namespace msensor {
class ICM20948 : public IImu {
public:
  struct xyz_data_ {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  struct double_xyz_data_ {
    double x;
    double y;
    double z;
  };

  // [2:1] bits
  enum GYRO_SCALE { DPS_2000 = 3, DPS_1000 = 2, DPS_500 = 1, DPS_250 = 0 };

  enum ACC_SCALE { G_16 = 3, G_8 = 2, G_4 = 1, G_2 = 0 };

  ICM20948(int i2c_device, int i2c_icm_address);
  virtual ~ICM20948() = default;

  bool init() const;

  // Performs a simple calibration. Device must be steady. No need to do it
  // everytime. Calibration values are written to the IMU.
  bool calibrate() const;

  std::optional<IMUData> getImuData() override;

private:
  xyz_data_ get_acc_data() const;
  xyz_data_ get_gyro_data() const;

  double_xyz_data_ convert_raw_data(const xyz_data_ &data, double factor) const;

private:
  void bank_select(int bank) const;

  // Register operations
  uint8_t write_(uint8_t reg_address, uint8_t data) const;
  uint8_t read_(uint8_t reg_address) const;
  // Return number of read bytes.
  int read_block_(uint8_t reg_address, int length, uint8_t *data) const;

  GYRO_SCALE get_gyro_scale() const;

  ACC_SCALE get_acc_scale() const;

  uint8_t get_gyro_offset_scale(GYRO_SCALE scale) const {
    switch (scale) {
    case DPS_2000:
      return 1;
    case DPS_1000:
      return 2;
    case DPS_500:
      return 4;
    case DPS_250:
      return 8;
    default:
      return 0;
    }
  }

  uint8_t get_acc_offset_scale(ACC_SCALE scale) const {
    switch (scale) {
    case G_16:
      return 1;
    case G_8:
      return 2;
    case G_4:
      return 4;
    case G_2:
      return 8;
    default:
      return 0;
    }
  }

  uint8_t get_acc_gravity_offset_scale(ACC_SCALE scale) const {
    switch (scale) {
    case G_16:
      return 16;
    case G_8:
      return 8;
    case G_4:
      return 4;
    case G_2:
      return 2;
    default:
      return 0;
    }
  }

  const int i2c_device_;
  const int i2c_icm_address_;
  int i2c_device_fd_;
  
};

}