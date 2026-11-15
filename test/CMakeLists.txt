find_package(GTest REQUIRED)

# Compiles with the legacy global names enabled, so it must be added before
# MSENSOR_NO_LEGACY_NAMES is set for this directory below.
add_subdirectory(legacy)

# msensor's own tests and benchmark are built without the legacy global names
# (see MSENSOR_NO_LEGACY_NAMES in AGENTS.md). Directory scope only.
add_compile_definitions(MSENSOR_NO_LEGACY_NAMES)

add_executable(test_recorder src/test_recorder.cc)
target_include_directories(test_recorder PRIVATE mocks)
target_link_libraries(test_recorder msensor::scan_recorder msensor::file gtest_main gtest gmock)
gtest_discover_tests(test_recorder)

add_executable(test_client_server src/test_client_server.cc)
target_include_directories(test_client_server PRIVATE mocks)
target_link_libraries(test_client_server msensor::server msensor::bus_recorder msensor::scan_recorder msensor::sim_lidar msensor::sim_imu msensor::sim_camera gtest_main gtest gmock)
gtest_discover_tests(test_client_server)

add_executable(test_conversions src/test_conversions.cc)
target_link_libraries(test_conversions msensor::conversions gtest_main gtest)
gtest_discover_tests(test_conversions)

add_executable(test_sensor_stream src/test_sensor_stream.cc)
target_link_libraries(test_sensor_stream msensor::server gtest_main gtest)
gtest_discover_tests(test_sensor_stream)

add_executable(test_sensor_bus src/test_sensor_bus.cc)
target_link_libraries(test_sensor_bus msensor::bus gtest_main gtest)
gtest_discover_tests(test_sensor_bus)

add_executable(test_bus_recorder src/test_bus_recorder.cc)
target_link_libraries(test_bus_recorder msensor::bus_recorder msensor::config msensor::scan_recorder msensor::file gtest_main gtest)
gtest_discover_tests(test_bus_recorder)

add_executable(test_sequence_gap_counter src/test_sequence_gap_counter.cc)
target_link_libraries(test_sequence_gap_counter msensor::server gtest_main gtest)
gtest_discover_tests(test_sequence_gap_counter)

add_executable(test_recording_driver src/test_recording_driver.cc)
target_link_libraries(test_recording_driver msensor::scan_recorder msensor::file gtest_main gtest)
gtest_discover_tests(test_recording_driver)

add_executable(test_sim_drivers src/test_sim_drivers.cc)
target_link_libraries(test_sim_drivers msensor::sim_camera msensor::sim_lidar msensor::sim_adc gtest_main gtest)
gtest_discover_tests(test_sim_drivers)

add_executable(test_sampling_thread src/test_sampling_thread.cc)
target_link_libraries(test_sampling_thread msensor::ILidar gtest_main gtest)
gtest_discover_tests(test_sampling_thread)

add_executable(test_rp_lidar src/test_rp_lidar.cc)
target_link_libraries(test_rp_lidar msensor::rp_lidar gtest_main gtest)
gtest_discover_tests(test_rp_lidar)

add_executable(test_shutdown_signal src/test_shutdown_signal.cc)
target_link_libraries(test_shutdown_signal msensor::server gtest_main gtest)
gtest_discover_tests(test_shutdown_signal)

add_executable(test_config src/test_config.cc)
target_link_libraries(test_config msensor::config gtest_main gtest)
target_compile_definitions(test_config PRIVATE MSENSOR_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
gtest_discover_tests(test_config)

add_executable(test_sensor_factory src/test_sensor_factory.cc)
target_link_libraries(test_sensor_factory msensor::sensor_factory gtest_main gtest)
gtest_discover_tests(test_sensor_factory)

add_executable(test_log src/test_log.cc)
target_link_libraries(test_log msensor::interface gtest_main gtest)
gtest_discover_tests(test_log)

add_executable(draft draft.cc)
target_link_libraries(draft msensor::scan_recorder)

# Benchmark of the conversion, playback, recording and gRPC paths; not a test.
# Run: bench_msensor [--passes N] [recording.pbscan ...]
add_executable(bench_msensor bench_msensor.cc)
target_link_libraries(bench_msensor msensor::server msensor::scan_recorder msensor::file)
