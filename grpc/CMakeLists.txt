# Sensor gRPC server

add_library(msensor_server
msensor_server.cc
lidar_service.cc
imu_service.cc
camera_service.cc
adc_service.cc
sensors_remote_client.cc)

target_link_libraries(msensor_server sensors_proto sensors_grpc IImu ILidar ICamera msensor_conversions)
target_include_directories(msensor_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(msensor::server ALIAS msensor_server)