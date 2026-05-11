cmake_minimum_required(VERSION 3.20)
project(msensor)

# Only override project-level defaults when building standalone
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

find_package(PCL 1.15.1 REQUIRED COMPONENTS common io filters) # for point types, point cloud serialization
find_package(OpenCV 4.10 REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(findGRPC)
include(generate_proto)

option (FORMAT_CODE "Format code" OFF)
if(FORMAT_CODE)
  include(format)
endif()

add_subdirectory(include)
add_subdirectory(third_party)
add_subdirectory(proto)
add_subdirectory(grpc)
add_subdirectory(src)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/mid360_config.json
              ${CMAKE_CURRENT_SOURCE_DIR}/cfg/publisher_config.json
DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

include(CTest)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
add_subdirectory(test)
endif()
