add_library(Camera
opencv_camera.cc)

target_link_libraries(Camera ICamera)