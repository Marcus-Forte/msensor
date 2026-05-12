add_library(scan_recorder
scan_player.cc
scan_recorder.cc)
target_link_libraries(scan_recorder sensors_proto ILidar timing file msensor::conversions)

add_library(msensor::scan_recorder ALIAS scan_recorder)

add_executable(scan_checker
scan_checker.cc)
target_link_libraries(scan_checker scan_recorder msensor::conversions)

install(TARGETS scan_checker 
DESTINATION bin)