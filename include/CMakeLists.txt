# Interface library definitions

add_library(IFile INTERFACE)
target_include_directories(IFile INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(IImu INTERFACE)
target_include_directories(IImu INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IImu INTERFACE timing)

add_library(ILidar INTERFACE)
target_include_directories(ILidar INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ILidar INTERFACE timing ${PCL_LIBRARIES})

add_library(IAdc INTERFACE)
target_include_directories(IAdc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ICamera INTERFACE)
target_include_directories(ICamera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ICamera INTERFACE ${OpenCV_LIBS})
target_include_directories(ICamera INTERFACE ${OpenCV_INCLUDE_DIRS})
