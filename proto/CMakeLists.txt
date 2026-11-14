# Sensor data proto libs.
# Each sensor has its own .proto file and generates separate proto + gRPC libraries.
# Real targets are msensor_<name>; each has an msensor::<name> alias.

# Generate protobuf and gRPC code for each proto file
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/header.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/lidar.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/imu.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/camera.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/adc.proto)
generate_proto(${CMAKE_CURRENT_SOURCE_DIR}/recording.proto)

# --- Header ---
add_library(msensor_header_proto ${CMAKE_CURRENT_BINARY_DIR}/header.pb.cc)
add_library(msensor::header_proto ALIAS msensor_header_proto)
target_include_directories(msensor_header_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_header_proto ${_PROTOBUF_LIBPROTOBUF})

# --- Lidar ---
add_library(msensor_lidar_proto ${CMAKE_CURRENT_BINARY_DIR}/lidar.pb.cc)
add_library(msensor::lidar_proto ALIAS msensor_lidar_proto)
target_include_directories(msensor_lidar_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_lidar_proto msensor::header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(msensor_lidar_grpc ${CMAKE_CURRENT_BINARY_DIR}/lidar.grpc.pb.cc)
add_library(msensor::lidar_grpc ALIAS msensor_lidar_grpc)
target_include_directories(msensor_lidar_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_lidar_grpc msensor::header_proto ${_GRPC_GRPCPP} msensor::lidar_proto)

# --- IMU ---
add_library(msensor_imu_proto ${CMAKE_CURRENT_BINARY_DIR}/imu.pb.cc)
add_library(msensor::imu_proto ALIAS msensor_imu_proto)
target_include_directories(msensor_imu_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_imu_proto msensor::header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(msensor_imu_grpc ${CMAKE_CURRENT_BINARY_DIR}/imu.grpc.pb.cc)
add_library(msensor::imu_grpc ALIAS msensor_imu_grpc)
target_include_directories(msensor_imu_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_imu_grpc msensor::header_proto ${_GRPC_GRPCPP} msensor::imu_proto)

# --- Camera ---
add_library(msensor_camera_proto ${CMAKE_CURRENT_BINARY_DIR}/camera.pb.cc)
add_library(msensor::camera_proto ALIAS msensor_camera_proto)
target_include_directories(msensor_camera_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_camera_proto msensor::header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(msensor_camera_grpc ${CMAKE_CURRENT_BINARY_DIR}/camera.grpc.pb.cc)
add_library(msensor::camera_grpc ALIAS msensor_camera_grpc)
target_include_directories(msensor_camera_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_camera_grpc msensor::header_proto ${_GRPC_GRPCPP} msensor::camera_proto)

# --- ADC ---
add_library(msensor_adc_proto ${CMAKE_CURRENT_BINARY_DIR}/adc.pb.cc)
add_library(msensor::adc_proto ALIAS msensor_adc_proto)
target_include_directories(msensor_adc_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_adc_proto msensor::header_proto ${_PROTOBUF_LIBPROTOBUF})

add_library(msensor_adc_grpc ${CMAKE_CURRENT_BINARY_DIR}/adc.grpc.pb.cc)
add_library(msensor::adc_grpc ALIAS msensor_adc_grpc)
target_include_directories(msensor_adc_grpc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_adc_grpc msensor::header_proto ${_GRPC_GRPCPP} msensor::adc_proto)

# --- Recording (depends on lidar + imu) ---
add_library(msensor_recording_proto ${CMAKE_CURRENT_BINARY_DIR}/recording.pb.cc)
add_library(msensor::recording_proto ALIAS msensor_recording_proto)
target_include_directories(msensor_recording_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(msensor_recording_proto ${_PROTOBUF_LIBPROTOBUF} msensor::lidar_proto msensor::imu_proto)

# Convenience targets that aggregate all proto/grpc libs
add_library(msensor_sensors_proto INTERFACE)
add_library(msensor::sensors_proto ALIAS msensor_sensors_proto)
target_link_libraries(msensor_sensors_proto INTERFACE msensor::lidar_proto msensor::imu_proto msensor::camera_proto msensor::adc_proto msensor::recording_proto)

add_library(msensor_sensors_grpc INTERFACE)
add_library(msensor::sensors_grpc ALIAS msensor_sensors_grpc)
target_link_libraries(msensor_sensors_grpc INTERFACE msensor::lidar_grpc msensor::imu_grpc msensor::camera_grpc msensor::adc_grpc)
