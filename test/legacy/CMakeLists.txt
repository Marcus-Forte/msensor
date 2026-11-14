# Proves the legacy global names (SensorsServer, toProtobuf, timing::, ...)
# still compile. Deliberately NOT built with MSENSOR_NO_LEGACY_NAMES; it only
# compiles and links, it never starts a server.
add_executable(test_legacy_names ../src/test_legacy_names.cc)
target_link_libraries(test_legacy_names msensor::server msensor::sim_lidar gtest_main gtest)
gtest_discover_tests(test_legacy_names)
