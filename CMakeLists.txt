cmake_minimum_required(VERSION 3.20)
project(msensor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL 1.15.1 REQUIRED COMPONENTS common io) # for point types, point cloud serialization
find_package(OpenCV 4.10 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(findGRPC)
include(generate_proto)

option (FORMAT_CODE "Format code" OFF)
option (IMPORT_ALL_RUNTIME "Import all runtime libraries, including system libraries like libc" OFF)
if(FORMAT_CODE)
  include(format)
endif()

add_subdirectory(include)
add_subdirectory(third_party)
add_subdirectory(proto)
add_subdirectory(grpc)
add_subdirectory(src)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/mid360_config.json
DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()
