add_library(msensor_icm20948
icm-20948.cc)
add_library(msensor::icm20948 ALIAS msensor_icm20948)
target_link_libraries(msensor_icm20948 i2c msensor::IImu)

add_library(msensor_sim_imu
sim_imu.cc)
add_library(msensor::sim_imu ALIAS msensor_sim_imu)
target_link_libraries(msensor_sim_imu msensor::IImu)
