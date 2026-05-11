find_package(GTest REQUIRED)

add_executable(test_recorder src/test_recorder.cc)
target_include_directories(test_recorder PRIVATE mocks)
target_link_libraries(test_recorder scan_recorder gtest_main gtest gmock)
gtest_discover_tests(test_recorder)

add_executable(test_client_server src/test_client_server.cc)
target_include_directories(test_client_server PRIVATE mocks)
target_link_libraries(test_client_server sensors_server gtest_main gtest gmock)
gtest_discover_tests(test_client_server)

add_executable(draft draft.cc)
target_link_libraries(draft scan_recorder)