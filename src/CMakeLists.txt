# Everything below is msensor's own code: build it without the legacy global
# names (see MSENSOR_NO_LEGACY_NAMES in AGENTS.md). Directory scope only; this
# is not propagated to consumers.
add_compile_definitions(MSENSOR_NO_LEGACY_NAMES)

add_subdirectory(apps)
add_subdirectory(file)
add_subdirectory(config)
add_subdirectory(conversions)
add_subdirectory(timing)
add_subdirectory(lidar)
add_subdirectory(imu)
add_subdirectory(recorder)
add_subdirectory(adc)
add_subdirectory(camera)

# Core library
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE msensor::timing msensor::server msensor::scan_recorder msensor::sensors_proto)

add_library(msensor::msensor ALIAS ${PROJECT_NAME})
