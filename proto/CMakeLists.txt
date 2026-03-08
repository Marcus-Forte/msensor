# Sensor data proto libs.
# Each sensor has its own .proto file and generates separate proto + gRPC libraries.

# Generate protobuf and gRPC code for each proto file
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/lidar.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/imu.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/camera.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/adc.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/recording.proto)

# --- Lidar ---
add_library(lidar_proto ${CMAKE_CURRENT_BINARY_DIR}/lidar.pb.cc)
target_include_directories(lidar_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(lidar_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(lidar_grpc ${CMAKE_CURRENT_BINARY_DIR}/lidar.grpc.pb.cc)
target_include_directories(lidar_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(lidar_grpc ${_GRPC_GRPCPP} lidar_proto)

# --- IMU ---
add_library(imu_proto ${CMAKE_CURRENT_BINARY_DIR}/imu.pb.cc)
target_include_directories(imu_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(imu_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(imu_grpc ${CMAKE_CURRENT_BINARY_DIR}/imu.grpc.pb.cc)
target_include_directories(imu_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(imu_grpc ${_GRPC_GRPCPP} imu_proto)

# --- Camera ---
add_library(camera_proto ${CMAKE_CURRENT_BINARY_DIR}/camera.pb.cc)
target_include_directories(camera_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(camera_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(camera_grpc ${CMAKE_CURRENT_BINARY_DIR}/camera.grpc.pb.cc)
target_include_directories(camera_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(camera_grpc ${_GRPC_GRPCPP} camera_proto)

# --- ADC ---
add_library(adc_proto ${CMAKE_CURRENT_BINARY_DIR}/adc.pb.cc)
target_include_directories(adc_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(adc_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(adc_grpc ${CMAKE_CURRENT_BINARY_DIR}/adc.grpc.pb.cc)
target_include_directories(adc_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(adc_grpc ${_GRPC_GRPCPP} adc_proto)

# --- Recording (depends on lidar + imu) ---
add_library(recording_proto ${CMAKE_CURRENT_BINARY_DIR}/recording.pb.cc)
target_include_directories(recording_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(recording_proto ${_PROTOBUF_LIBPROTOBUF} lidar_proto imu_proto)

# Convenience targets that aggregate all proto/grpc libs (for backward compat)
add_library(sensors_proto INTERFACE)
target_link_libraries(sensors_proto INTERFACE lidar_proto imu_proto camera_proto adc_proto recording_proto)

add_library(sensors_grpc INTERFACE)
target_link_libraries(sensors_grpc INTERFACE lidar_grpc imu_grpc camera_grpc adc_grpc)