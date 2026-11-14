find_package(GTest REQUIRED)

add_executable(test_recorder src/test_recorder.cc)
target_include_directories(test_recorder PRIVATE mocks)
target_link_libraries(test_recorder scan_recorder file gtest_main gtest gmock)
gtest_discover_tests(test_recorder)

add_executable(test_client_server src/test_client_server.cc)
target_include_directories(test_client_server PRIVATE mocks)
target_link_libraries(test_client_server msensor::server sim_lidar sim_imu gtest_main gtest gmock)
gtest_discover_tests(test_client_server)

add_executable(test_conversions src/test_conversions.cc)
target_link_libraries(test_conversions msensor::conversions gtest_main gtest)
gtest_discover_tests(test_conversions)

add_executable(test_sensor_stream src/test_sensor_stream.cc)
target_link_libraries(test_sensor_stream msensor::server gtest_main gtest)
gtest_discover_tests(test_sensor_stream)

add_executable(test_recording_driver src/test_recording_driver.cc)
target_link_libraries(test_recording_driver scan_recorder file gtest_main gtest)
gtest_discover_tests(test_recording_driver)

add_executable(test_sim_drivers src/test_sim_drivers.cc)
target_link_libraries(test_sim_drivers sim_camera sim_lidar sim_adc gtest_main gtest)
gtest_discover_tests(test_sim_drivers)

add_executable(draft draft.cc)
target_link_libraries(draft scan_recorder)

# Benchmark of the conversion, playback, recording and gRPC paths; not a test.
# Run: bench_msensor [--passes N] [recording.pbscan ...]
add_executable(bench_msensor bench_msensor.cc)
target_link_libraries(bench_msensor msensor::server scan_recorder file)
