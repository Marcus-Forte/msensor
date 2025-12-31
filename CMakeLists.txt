cmake_minimum_required(VERSION 3.20)
project(msensor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL 1.15.1 REQUIRED COMPONENTS common io) # for point types, point cloud serialization
find_package(OpenCV 4.10 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(findGRPC)
include(generate_proto)

add_subdirectory(include)
add_subdirectory(third_party)

option (FORMAT_CODE "Format code" OFF)
option (IMPORT_ALL_RUNTIME "Import all runtime libraries, including system libraries like libc" OFF)
if(FORMAT_CODE)
  include(format)
endif()


add_subdirectory(src/file)
add_subdirectory(src/timing)
add_subdirectory(src/lidar)
add_subdirectory(src/imu)
add_subdirectory(src/recorder)
add_subdirectory(src/adc)
add_subdirectory(src/camera)

add_subdirectory(proto)
add_subdirectory(grpc)

# Combines all
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_NAME} Timing sensors_server scan_recorder sensors_proto)

add_executable(icm20948_publisher src/icm20948_publisher.cc)
target_link_libraries(icm20948_publisher ${PROJECT_NAME} icm-20948)

add_executable(rplidar_publisher src/rplidar_publisher.cc)
target_link_libraries(rplidar_publisher RPLidar ${PROJECT_NAME})

add_executable(mid360_publisher src/mid360_publisher.cc)
target_link_libraries(mid360_publisher mid360 ${PROJECT_NAME})

add_executable(sim_publisher src/sim_publisher.cc)
target_link_libraries(sim_publisher ${PROJECT_NAME} sim_lidar sim_imu)

add_executable(ads1115_publisher src/ads1115_publisher.cc)
target_link_libraries(ads1115_publisher ads1115 ${PROJECT_NAME})

add_executable(camera_publisher src/camera_publisher.cc)
target_link_libraries(camera_publisher opencv_camera ${PROJECT_NAME})

add_executable(all_publisher src/all_publisher.cc)
target_link_libraries(all_publisher ads1115 RPLidar icm-20948 ${PROJECT_NAME})

include(GNUInstallDirs)
install(TARGETS all_publisher rplidar_publisher mid360_publisher sim_publisher icm20948_publisher ads1115_publisher camera_publisher
DESTINATION ${CMAKE_INSTALL_BINDIR} RUNTIME_DEPENDENCY_SET runtime_targets)

# Import runtime dependencies, exclude system libraries
set(RUNTIME_DEPENDENCY_POST_EXCLUDE_REGEX "libgst|libm|gcc_s|libc|ld-linux")
install(RUNTIME_DEPENDENCY_SET runtime_targets POST_EXCLUDE_REGEXES ${RUNTIME_DEPENDENCY_POST_EXCLUDE_REGEX} PRE_INCLUDE_REGEXES "."
DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/mid360_config.json
DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})

include(CTest)
if(BUILD_TESTING)
add_subdirectory(test)
endif()
