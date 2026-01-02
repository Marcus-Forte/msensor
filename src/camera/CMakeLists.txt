add_library(opencv_camera
opencv_camera.cc)
target_link_libraries(opencv_camera ICamera timing)


add_library(sim_camera
sim_camera.cc)
target_link_libraries(sim_camera ICamera timing)