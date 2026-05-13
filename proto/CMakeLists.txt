# Sensor data proto libs.
# Each sensor has its own .proto file and generates separate proto + gRPC libraries.

# Generate protobuf and gRPC code for each proto file
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/header.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/lidar.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/imu.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/camera.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/adc.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/recording.proto)

# --- Header ---
add_library(header_proto ${CMAKE_CURRENT_BINARY_DIR}/header.pb.cc)
target_include_directories(header_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(header_proto ${_PROTOBUF_LIBPROTOBUF})

# --- Lidar ---
add_library(lidar_proto ${CMAKE_CURRENT_BINARY_DIR}/lidar.pb.cc)
target_include_directories(lidar_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(lidar_proto header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(lidar_grpc ${CMAKE_CURRENT_BINARY_DIR}/lidar.grpc.pb.cc)
target_include_directories(lidar_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(lidar_grpc header_proto ${_GRPC_GRPCPP} lidar_proto)

# --- IMU ---
add_library(imu_proto ${CMAKE_CURRENT_BINARY_DIR}/imu.pb.cc)
target_include_directories(imu_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(imu_proto header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(imu_grpc ${CMAKE_CURRENT_BINARY_DIR}/imu.grpc.pb.cc)
target_include_directories(imu_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(imu_grpc header_proto ${_GRPC_GRPCPP} imu_proto)

# --- Camera ---
add_library(camera_proto ${CMAKE_CURRENT_BINARY_DIR}/camera.pb.cc)
target_include_directories(camera_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(camera_proto header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(camera_grpc ${CMAKE_CURRENT_BINARY_DIR}/camera.grpc.pb.cc)
target_include_directories(camera_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(camera_grpc header_proto ${_GRPC_GRPCPP} camera_proto)

# --- ADC ---
add_library(adc_proto ${CMAKE_CURRENT_BINARY_DIR}/adc.pb.cc)
target_include_directories(adc_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(adc_proto header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(adc_grpc ${CMAKE_CURRENT_BINARY_DIR}/adc.grpc.pb.cc)
target_include_directories(adc_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(adc_grpc header_proto ${_GRPC_GRPCPP} adc_proto)

# --- Recording (depends on lidar + imu) ---
add_library(recording_proto ${CMAKE_CURRENT_BINARY_DIR}/recording.pb.cc)
target_include_directories(recording_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(recording_proto ${_PROTOBUF_LIBPROTOBUF} lidar_proto imu_proto)

# Convenience targets that aggregate all proto/grpc libs (for backward compat)
add_library(sensors_proto INTERFACE)
target_link_libraries(sensors_proto INTERFACE lidar_proto imu_proto camera_proto adc_proto recording_proto)

add_library(sensors_grpc INTERFACE)
target_link_libraries(sensors_grpc INTERFACE lidar_grpc imu_grpc camera_grpc adc_grpc)

# Namespaced aliases for submodule consumers
add_library(msensor::lidar_proto ALIAS lidar_proto)
add_library(msensor::lidar_grpc ALIAS lidar_grpc)
add_library(msensor::imu_proto ALIAS imu_proto)
add_library(msensor::imu_grpc ALIAS imu_grpc)
add_library(msensor::camera_proto ALIAS camera_proto)
add_library(msensor::camera_grpc ALIAS camera_grpc)
add_library(msensor::adc_proto ALIAS adc_proto)
add_library(msensor::adc_grpc ALIAS adc_grpc)
add_library(msensor::recording_proto ALIAS recording_proto)
add_library(msensor::sensors_proto ALIAS sensors_proto)
add_library(msensor::sensors_grpc ALIAS sensors_grpc)