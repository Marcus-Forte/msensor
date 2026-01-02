link_libraries(ILidar)

add_library(sim_lidar
sim_lidar.cc)

add_library(mid360
mid360.cc)
target_link_libraries(mid360 livox_lidar_sdk_static)

add_library(rp_lidar 
  rp_lidar.cc)
target_link_libraries(rp_lidar rplidar_sdk ILidar)