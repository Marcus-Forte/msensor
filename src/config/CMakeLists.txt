add_library(config config.cc)

target_link_libraries(config IConfig JsonCpp::JsonCpp)
target_include_directories(
	config
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)