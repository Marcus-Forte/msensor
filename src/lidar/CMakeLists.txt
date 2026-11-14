add_library(msensor_sim_lidar
sim_lidar.cc)
add_library(msensor::sim_lidar ALIAS msensor_sim_lidar)
target_link_libraries(msensor_sim_lidar msensor::ILidar)

add_library(msensor_mid360
mid360.cc)
add_library(msensor::mid360 ALIAS msensor_mid360)
target_link_libraries(msensor_mid360 livox_lidar_sdk_static msensor::ILidar)

add_library(msensor_rp_lidar
  rp_lidar.cc)
add_library(msensor::rp_lidar ALIAS msensor_rp_lidar)
target_link_libraries(msensor_rp_lidar rplidar_sdk msensor::ILidar)
