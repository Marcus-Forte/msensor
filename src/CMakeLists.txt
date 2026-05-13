
add_subdirectory(apps)
add_subdirectory(file)
add_subdirectory(config)
add_subdirectory(conversions)
add_subdirectory(timing)
add_subdirectory(lidar)
add_subdirectory(imu)
add_subdirectory(recorder)
add_subdirectory(adc)
add_subdirectory(camera)

# Core library
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE timing msensor::server scan_recorder sensors_proto)

add_library(msensor::msensor ALIAS ${PROJECT_NAME})