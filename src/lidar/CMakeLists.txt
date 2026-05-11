add_library(sim_lidar
sim_lidar.cc)
target_link_libraries(sim_lidar ILidar)

add_library(mid360
mid360.cc)
target_link_libraries(mid360 livox_lidar_sdk_static ILidar)

add_library(rp_lidar 
  rp_lidar.cc)
target_link_libraries(rp_lidar rplidar_sdk ILidar)