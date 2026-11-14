# Applications

# Sensor simulator
add_executable(sim_publisher sim_publisher.cc)
target_link_libraries(sim_publisher msensor::msensor msensor::sensor_factory msensor::config)

# Publishers
add_executable(icm20948_publisher icm20948_publisher.cc)
target_link_libraries(icm20948_publisher msensor::msensor msensor::icm20948)

add_executable(rplidar_publisher rplidar_publisher.cc)
target_link_libraries(rplidar_publisher msensor::rp_lidar msensor::msensor)

add_executable(mid360_publisher mid360_publisher.cc)
target_link_libraries(mid360_publisher msensor::mid360 msensor::msensor)

add_executable(ads1115_publisher ads1115_publisher.cc)
target_link_libraries(ads1115_publisher msensor::ads1115 msensor::msensor)

add_executable(camera_publisher camera_publisher.cc)
target_link_libraries(camera_publisher msensor::opencv_camera msensor::msensor)

add_executable(sensor_publisher sensor_publisher.cc)
target_link_libraries(sensor_publisher msensor::msensor msensor::sensor_factory msensor::config)

add_executable(remote_recorder remote_recorder.cc)
target_link_libraries(remote_recorder msensor::msensor)

# Replays a recorded .pbscan over gRPC as a sensor source.
add_executable(playback_publisher playback_publisher.cc)
target_link_libraries(playback_publisher msensor::msensor)

include(GNUInstallDirs)
set(msensor_install_targets
    sensor_publisher
    remote_recorder
    rplidar_publisher
    mid360_publisher
    sim_publisher
    icm20948_publisher
    ads1115_publisher
    camera_publisher
)
if(MSENSOR_INSTALL_PLAYBACK_PUBLISHER AND TARGET playback_publisher)
    list(APPEND msensor_install_targets playback_publisher)
endif()

install(TARGETS ${msensor_install_targets}
DESTINATION ${CMAKE_INSTALL_BINDIR} RUNTIME_DEPENDENCY_SET runtime_targets)

# Import runtime dependencies while excluding system libraries from /lib and
# /usr/lib.
install(RUNTIME_DEPENDENCY_SET runtime_targets
    PRE_INCLUDE_REGEXES "."
    POST_EXCLUDE_REGEXES "^/lib/.*" "^/usr/lib/.*"
    DESTINATION ${CMAKE_INSTALL_LIBDIR})
