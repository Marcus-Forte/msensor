add_library(timing timing.cc)
target_include_directories(timing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)


add_library(msensor::timing ALIAS timing)