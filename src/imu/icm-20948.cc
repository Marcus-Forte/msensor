
#include <fcntl.h>
extern "C" {
#include <i2c/smbus.h>
}
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include "msensor/imu/icm-20948.h"
#include "msensor/imu/icm-20948_defs.h"
#include "msensor/timing/timing.hh"

namespace msensor {

inline uint8_t ICM20948::write_(uint8_t reg_address, uint8_t data) const {
  return i2c_smbus_write_byte_data(i2c_device_fd_, reg_address, data);
}
inline uint8_t ICM20948::read_(uint8_t reg_address) const {
  return i2c_smbus_read_byte_data(i2c_device_fd_, reg_address);
}

inline int ICM20948::read_block_(uint8_t reg_address, int length,
                                 uint8_t *data) const {
  return i2c_smbus_read_i2c_block_data(i2c_device_fd_, reg_address, length,
                                       data);
}

void ICM20948::bank_select(int bank) const {
  uint8_t data;
  switch (bank) {
  case 0:
    data = 0b00000000;
    break;
  case 1:
    data = 0b00010000;
    break;
  case 2:
    data = 0b00100000;
    break;
  case 3:
    data = 0b00100000;
    break;
  }
  int res = write_(REG_REG_BANK_SEL, data);

  if (res != 0)
    throw std::runtime_error("Unable to write to Bank selector.");
}

ICM20948::ICM20948(int i2c_device, int i2c_icm_address)
    : i2c_device_(i2c_device), i2c_icm_address_(i2c_icm_address) {
  const std::string i2c_device_file = "/dev/i2c-" + std::to_string(i2c_device);
  i2c_device_fd_ = open(i2c_device_file.c_str(), O_RDWR);

  if (i2c_device_fd_ < 0) {
    throw std::runtime_error("unable to open I2C (system) device: " +
                             std::to_string(i2c_device));
  }

  if (ioctl(i2c_device_fd_, I2C_SLAVE, i2c_icm_address) < 0) {
    throw std::runtime_error("unable to open IMU device: " +
                             std::to_string(i2c_icm_address)); // to hex?
  }
}

/// \todo allow configuration.
bool ICM20948::init() const {
  bank_select(0);
  // Read device ID. We are in BANK0
  auto who_am_i_data = read_(REG_WHO_AM_I);
  if (who_am_i_data != CONST_WHO_AM_I_VALUE) {
    const std::string error_message =
        "Unexpected WHO_AM_I value\nExpected:" +
        std::to_string(CONST_WHO_AM_I_VALUE) +
        "\nReceived: " + std::to_string(who_am_i_data);
    throw std::runtime_error(error_message); // to hex?
  }

  // NO FIFO
  write_(REG_USER_CTRL, 0b00000000);
  // CLK_SEL = 1
  write_(REG_PWR_MGMT_1, 0b00000001);
  // Turn Gyro and Accel ON.
  write_(REG_PWR_MGMT_2, 0b00000000);

  bank_select(2);

  // [2:1] 500 dps/s scale. No digital filter.
  write_(REG_GYRO_CONFIG_1, 0b00000010);

  // [2:1] 2g scale. No digital filter.
  write_(REG_ACCEL_CONFIG_1, 0b00000000);

  // FIFO mode stream
  write_(REG_FIFO_MODE, 0b00000000);

  bank_select(0); // Important.
  return true;
}

bool ICM20948::calibrate() const {
  // set calibration ioffsets to zero.
  bank_select(1);

  write_(REG_XA_OFFS_H, 0x00);
  write_(REG_XA_OFFS_L, 0x00);
  write_(REG_YA_OFFS_H, 0x00);
  write_(REG_YA_OFFS_L, 0x00);
  write_(REG_ZA_OFFS_H, 0x00);
  write_(REG_ZA_OFFS_L, 0x00);

  bank_select(2);

  write_(REG_XG_OFFS_USRH, 0x00);
  write_(REG_XG_OFFS_USRL, 0x00);
  write_(REG_YG_OFFS_USRH, 0x00);
  write_(REG_YG_OFFS_USRL, 0x00);
  write_(REG_ZG_OFFS_USRH, 0x00);
  write_(REG_ZG_OFFS_USRL, 0x00);

  // Get scales
  auto acc_scale = get_acc_scale();
  uint8_t acc_gravity_scale = get_acc_gravity_offset_scale(acc_scale);
  uint8_t acc_scale_offset_factor = get_acc_offset_scale(acc_scale);
  auto gyro_scale = get_gyro_scale();
  auto gyro_offset_factor = get_gyro_offset_scale(gyro_scale);

  printf("gyro factor = %d\n", gyro_offset_factor);
  printf("acc_gravity_scale factor = %d\n", acc_gravity_scale);
  printf("acc_scale_offset_factor factor = %d\n", acc_scale_offset_factor);
  bank_select(0);

  // calibration configuration
  const int calibration_samples = 128;
  int read_samples = 0;
  const int us_period = 2000;

  xyz_data_ acc_calib_data;
  xyz_data_ gyr_calib_data;
  int32_t gyr_offset[3] = {0};
  int32_t acc_offset[3] = {0};

  while (read_samples < calibration_samples) {
    acc_calib_data = get_acc_data();
    gyr_calib_data = get_gyro_data();

    gyr_offset[0] -= gyr_calib_data.x;
    gyr_offset[1] -= gyr_calib_data.y;
    gyr_offset[2] -= gyr_calib_data.z;

    acc_offset[0] -= acc_calib_data.x;
    acc_offset[1] -= acc_calib_data.y;
    acc_offset[2] -= acc_calib_data.z;

    read_samples++;
    usleep(us_period);
  }

  gyr_offset[0] /= calibration_samples;
  gyr_offset[1] /= calibration_samples;
  gyr_offset[2] /= calibration_samples;

  acc_offset[0] /= calibration_samples;
  acc_offset[1] /= calibration_samples;
  acc_offset[2] /= calibration_samples;

  // Add gravity. Assuming IMU is flat.
  acc_offset[2] += 32767 / acc_gravity_scale;

  printf("Acc scale: %d", acc_scale);
  printf("scales: %d , %d\n\n", acc_gravity_scale, acc_scale_offset_factor);
  // Use full scale
  // Reduce to full scale (divide) and apply one's compliement.  // Those are 15
  // bit registers.
  acc_offset[0] = ((acc_offset[0] / acc_scale_offset_factor) & ~1);
  acc_offset[1] = ((acc_offset[1] / acc_scale_offset_factor) & ~1);
  acc_offset[2] = ((acc_offset[2] / acc_scale_offset_factor) & ~1);

  gyr_offset[0] = gyr_offset[0] / gyro_offset_factor;
  gyr_offset[1] = gyr_offset[1] / gyro_offset_factor;
  gyr_offset[2] = gyr_offset[2] / gyro_offset_factor;

  bank_select(2);
  write_(REG_XG_OFFS_USRH, (gyr_offset[0] & 0xFF00) >> 8);
  write_(REG_XG_OFFS_USRL, (gyr_offset[0] & 0x00FF));
  write_(REG_YG_OFFS_USRH, (gyr_offset[1] & 0xFF00) >> 8);
  write_(REG_YG_OFFS_USRL, (gyr_offset[1] & 0x00FF));
  write_(REG_ZG_OFFS_USRH, (gyr_offset[2] & 0xFF00) >> 8);
  write_(REG_ZG_OFFS_USRL, (gyr_offset[2] & 0x00FF));
  bank_select(1);
  write_(REG_XA_OFFS_H, (acc_offset[0] >> 8) & 0xFF);
  write_(REG_XA_OFFS_L, (acc_offset[0] & 0xFF));
  write_(REG_YA_OFFS_H, (acc_offset[1] >> 8) & 0xFF);
  write_(REG_YA_OFFS_L, (acc_offset[1] & 0xFF));
  write_(REG_ZA_OFFS_H, (acc_offset[2] >> 8) & 0xFF);
  write_(REG_ZA_OFFS_L, (acc_offset[2] & 0xFF));
  bank_select(0);

  return true;
}

std::optional<IMUData> ICM20948::getImuData() {
  static uint32_t sequence_number = 0;
  auto acc_data = get_acc_data();
  auto gyr_data = get_gyro_data();
  auto dbl_acc_data = convert_raw_data(acc_data, FACTOR_ACC_2G);
  auto dbl_gyr_data = convert_raw_data(gyr_data, FACTOR_GYRO_500DPS_RADS);

  return {IMUData{
      Header{timing::getNowNs(), sequence_number++},
      static_cast<float>(dbl_acc_data.x), static_cast<float>(dbl_acc_data.y),
      static_cast<float>(dbl_acc_data.z), static_cast<float>(dbl_gyr_data.x),
      static_cast<float>(dbl_gyr_data.y), static_cast<float>(dbl_gyr_data.z)}};
}

ICM20948::xyz_data_ ICM20948::get_acc_data() const {
  uint8_t data[6];
  auto bytes_read = read_block_(REG_ACCEL_XOUT_H, 6, data);
  if (bytes_read != 6)
    throw std::runtime_error(
        "error getting data from accelerometer"); // to hex?
  xyz_data_ ret;
  ret.x = data[0] << 8 | data[1];
  ret.y = data[2] << 8 | data[3];
  ret.z = data[4] << 8 | data[5];
  return ret;
}

ICM20948::xyz_data_ ICM20948::get_gyro_data() const {
  uint8_t data[6];
  auto bytes_read = read_block_(REG_GYRO_XOUT_H, 6, data);
  if (bytes_read != 6)
    throw std::runtime_error("error getting data from gyro"); // to hex?

  xyz_data_ ret;
  ret.x = data[0] << 8 | data[1];
  ret.y = data[2] << 8 | data[3];
  ret.z = data[4] << 8 | data[5];
  return ret;
}

ICM20948::double_xyz_data_ ICM20948::convert_raw_data(const xyz_data_ &data,
                                                      double factor) const {
  double_xyz_data_ ret;
  ret.x = (double)data.x * factor;
  ret.y = (double)data.y * factor;
  ret.z = (double)data.z * factor;

  return ret;
}

ICM20948::GYRO_SCALE ICM20948::get_gyro_scale() const {
  uint8_t scale_bits = read_(REG_GYRO_CONFIG_1);
  scale_bits = (scale_bits >> 1) & 0x03;
  return (GYRO_SCALE)scale_bits;
}

ICM20948::ACC_SCALE ICM20948::get_acc_scale() const {
  uint8_t scale_bits = read_(REG_ACCEL_CONFIG_1);
  scale_bits = (scale_bits >> 1) & 0x03;
  return (ACC_SCALE)scale_bits;
}
} // namespace msensor