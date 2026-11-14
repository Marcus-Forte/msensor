add_library(msensor_scan_recorder
scan_player.cc
scan_recorder.cc
recording_driver.cc)
add_library(msensor::scan_recorder ALIAS msensor_scan_recorder)
target_link_libraries(msensor_scan_recorder msensor::sensors_proto
  msensor::ILidar msensor::IImu msensor::timing
  msensor::file msensor::lidar_imu_conversions)

add_executable(scan_checker
scan_checker.cc)
target_link_libraries(scan_checker msensor::scan_recorder msensor::lidar_imu_conversions)

install(TARGETS scan_checker 
DESTINATION bin)
