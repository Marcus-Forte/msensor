
add_subdirectory(apps)
add_subdirectory(file)
add_subdirectory(config)
add_subdirectory(timing)
add_subdirectory(lidar)
add_subdirectory(imu)
add_subdirectory(recorder)
add_subdirectory(adc)
add_subdirectory(camera)

add_library(msensor_conversions conversions.cc)
target_link_libraries(msensor_conversions sensors_proto sensors_grpc IImu ILidar ICamera)
add_library(msensor::conversions ALIAS msensor_conversions)

# Core library
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE timing msensor::server scan_recorder sensors_proto)

add_library(msensor::msensor ALIAS ${PROJECT_NAME})