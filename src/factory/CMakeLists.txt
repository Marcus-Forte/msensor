add_library(msensor_sensor_factory sensor_factory.cc)
add_library(msensor::sensor_factory ALIAS msensor_sensor_factory)

target_link_libraries(msensor_sensor_factory
  PUBLIC msensor::config msensor::IAdc msensor::IImu msensor::ILidar msensor::ICamera
  PRIVATE msensor::ads1115 msensor::icm20948 msensor::mid360 msensor::rp_lidar
          msensor::opencv_camera msensor::sim_adc msensor::sim_imu
          msensor::sim_lidar msensor::sim_camera)
