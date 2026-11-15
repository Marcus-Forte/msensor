# Applications

# Sensor simulator
add_executable(sim_publisher sim_publisher.cc)
target_link_libraries(sim_publisher msensor::msensor msensor::sensor_factory msensor::config)

# Publishers
add_executable(sensor_publisher sensor_publisher.cc)
target_link_libraries(sensor_publisher msensor::msensor msensor::sensor_factory msensor::config msensor::bus_recorder)

add_executable(remote_recorder remote_recorder.cc)
target_link_libraries(remote_recorder msensor::msensor)

# Replays a recorded .pbscan over gRPC as a sensor source.
add_executable(playback_publisher playback_publisher.cc)
target_link_libraries(playback_publisher msensor::msensor)

include(GNUInstallDirs)
set(msensor_install_targets
    sensor_publisher
    remote_recorder
    sim_publisher
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
