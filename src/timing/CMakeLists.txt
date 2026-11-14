add_library(msensor_timing timing.cc)
target_include_directories(msensor_timing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

add_library(msensor::timing ALIAS msensor_timing)
