add_library(msensor_config config.cc)
add_library(msensor::config ALIAS msensor_config)

target_link_libraries(msensor_config msensor::IConfig JsonCpp::JsonCpp)
target_include_directories(
	msensor_config
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
