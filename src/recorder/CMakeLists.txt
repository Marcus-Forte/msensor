link_libraries(sensors_proto ILidar timing file)

add_library(scan_recorder
scan_player.cc
scan_recorder.cc)

add_executable(scan_checker
scan_checker.cc)
target_link_libraries(scan_checker scan_recorder)

install(TARGETS scan_checker 
DESTINATION bin)