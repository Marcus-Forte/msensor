# Sensor gRPC server

add_library(sensors_server
conversions.cc
sensors_server.cc
sensors_service.cc
sensors_remote_client.cc)

target_link_libraries(sensors_server sensors_proto sensors_grpc IImu ILidar ICamera Timing)
target_include_directories(sensors_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})