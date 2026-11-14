add_library(msensor_file
file.cc)
add_library(msensor::file ALIAS msensor_file)

target_link_libraries(msensor_file msensor::IFile)
