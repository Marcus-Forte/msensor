# Sensor gRPC server

# msensor's own code is built without the legacy global names (see
# MSENSOR_NO_LEGACY_NAMES in AGENTS.md). Directory scope only; not propagated.
add_compile_definitions(MSENSOR_NO_LEGACY_NAMES)

add_library(msensor_server
msensor_server.cc
lidar_service.cc
imu_service.cc
camera_service.cc
adc_service.cc
sensors_remote_client.cc
shutdown_signal.cc)

target_link_libraries(msensor_server msensor::sensors_proto msensor::sensors_grpc
  msensor::bus msensor::IImu msensor::ILidar msensor::ICamera
  msensor::lidar_imu_conversions msensor::camera_conversions)
target_include_directories(msensor_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(msensor::server ALIAS msensor_server)
