# LiDAR and IMU conversions: protobuf only, no gRPC and no OpenCV.
add_library(msensor_lidar_imu_conversions lidar_imu_conversions.cc)
add_library(msensor::lidar_imu_conversions ALIAS msensor_lidar_imu_conversions)
target_link_libraries(msensor_lidar_imu_conversions
  msensor::lidar_proto msensor::imu_proto msensor::ILidar msensor::IImu)

# Camera conversions: camera protobuf, camera interface and OpenCV.
add_library(msensor_camera_conversions camera_conversions.cc)
add_library(msensor::camera_conversions ALIAS msensor_camera_conversions)
target_link_libraries(msensor_camera_conversions
  msensor::camera_proto msensor::ICamera)

# Everything (what msensor_conversions used to provide).
add_library(msensor_conversions_all INTERFACE)
add_library(msensor::conversions ALIAS msensor_conversions_all)
add_library(msensor::conversions_all ALIAS msensor_conversions_all)
target_link_libraries(msensor_conversions_all INTERFACE
  msensor::lidar_imu_conversions msensor::camera_conversions)
