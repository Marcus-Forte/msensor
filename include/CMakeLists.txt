# Interface library definitions

add_library(IFile INTERFACE)
target_include_directories(IFile INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(IImu INTERFACE)
target_include_directories(IImu INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IImu INTERFACE Timing)

add_library(ILidar INTERFACE)
target_include_directories(ILidar INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ILidar INTERFACE Timing ${PCL_LIBRARIES})

add_library(IAdc INTERFACE)
target_include_directories(IAdc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ICamera INTERFACE)
target_include_directories(ICamera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ICamera INTERFACE ${OpenCV_LIBS})
