add_subdirectory(file)
add_subdirectory(timing)
add_subdirectory(lidar)
add_subdirectory(imu)
add_subdirectory(recorder)
add_subdirectory(adc)
add_subdirectory(camera)

# Core library
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME} timing sensors_server scan_recorder sensors_proto)

# Sensor simulator
add_executable(sim_publisher sim_publisher.cc)
target_link_libraries(sim_publisher ${PROJECT_NAME} sim_lidar sim_imu sim_camera sim_adc)

add_executable(icm20948_publisher icm20948_publisher.cc)
target_link_libraries(icm20948_publisher ${PROJECT_NAME} icm-20948)

add_executable(rplidar_publisher rplidar_publisher.cc)
target_link_libraries(rplidar_publisher rp_lidar ${PROJECT_NAME})

add_executable(mid360_publisher mid360_publisher.cc)
target_link_libraries(mid360_publisher mid360 ${PROJECT_NAME})

add_executable(ads1115_publisher ads1115_publisher.cc)
target_link_libraries(ads1115_publisher ads1115 ${PROJECT_NAME})

add_executable(camera_publisher camera_publisher.cc)
target_link_libraries(camera_publisher opencv_camera ${PROJECT_NAME})

add_executable(all_publisher all_publisher.cc)
target_link_libraries(all_publisher ads1115 rp_lidar icm-20948 ${PROJECT_NAME} opencv_camera)


include(GNUInstallDirs)
install(TARGETS 
    all_publisher
    rplidar_publisher
    mid360_publisher
    sim_publisher
    icm20948_publisher
    ads1115_publisher
    camera_publisher
DESTINATION ${CMAKE_INSTALL_BINDIR} RUNTIME_DEPENDENCY_SET runtime_targets)

# Import runtime dependencies, exclude system libraries
set(RUNTIME_DEPENDENCY_POST_EXCLUDE_REGEX "^/lib/.*")
install(RUNTIME_DEPENDENCY_SET runtime_targets POST_EXCLUDE_REGEXES ${RUNTIME_DEPENDENCY_POST_EXCLUDE_REGEX} PRE_INCLUDE_REGEXES "."
DESTINATION ${CMAKE_INSTALL_LIBDIR})