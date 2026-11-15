# Interface libraries.
#
# One header-only target serves every sensor interface that does not need
# OpenCV. The per-interface names (msensor::IFile, IConfig, IImu, ILidar, IAdc)
# are aliases of it.
add_library(msensor_interface INTERFACE)
target_include_directories(msensor_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(msensor_interface INTERFACE msensor::timing)

add_library(msensor::interface ALIAS msensor_interface)
add_library(msensor::IFile ALIAS msensor_interface)
add_library(msensor::IConfig ALIAS msensor_interface)
add_library(msensor::IImu ALIAS msensor_interface)
add_library(msensor::ILidar ALIAS msensor_interface)
add_library(msensor::IAdc ALIAS msensor_interface)

# The camera interface carries OpenCV, so it stays separate.
add_library(msensor_camera_interface INTERFACE)
target_include_directories(msensor_camera_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(msensor_camera_interface INTERFACE ${OpenCV_LIBS})
target_include_directories(msensor_camera_interface INTERFACE ${OpenCV_INCLUDE_DIRS})

add_library(msensor::camera_interface ALIAS msensor_camera_interface)
add_library(msensor::ICamera ALIAS msensor_camera_interface)

# In-process sensor bus (Topic, Subscription, SensorBus). Header-only; it needs
# the sensor interfaces (OpenCV through the camera one) and a thread library.
find_package(Threads REQUIRED)
add_library(msensor_bus INTERFACE)
target_include_directories(msensor_bus INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(msensor_bus INTERFACE msensor::interface msensor::camera_interface Threads::Threads)

add_library(msensor::bus ALIAS msensor_bus)
