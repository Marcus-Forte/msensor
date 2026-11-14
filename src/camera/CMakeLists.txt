add_library(msensor_opencv_camera
opencv_camera.cc)
add_library(msensor::opencv_camera ALIAS msensor_opencv_camera)
target_link_libraries(msensor_opencv_camera msensor::ICamera msensor::timing)


add_library(msensor_sim_camera
sim_camera.cc)
add_library(msensor::sim_camera ALIAS msensor_sim_camera)
target_link_libraries(msensor_sim_camera msensor::ICamera msensor::timing)
