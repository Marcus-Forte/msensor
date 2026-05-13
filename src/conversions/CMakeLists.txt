
add_library(msensor_conversions conversions.cc)
target_link_libraries(msensor_conversions sensors_proto sensors_grpc IImu ILidar ICamera)
add_library(msensor::conversions ALIAS msensor_conversions)
