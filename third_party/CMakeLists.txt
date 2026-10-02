
set(_msensor_cxx_flags "${CMAKE_CXX_FLAGS}")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  string(APPEND CMAKE_CXX_FLAGS " -fchar8_t -Wno-pragmas")
endif()

add_subdirectory(Livox-SDK2)

set(CMAKE_CXX_FLAGS "${_msensor_cxx_flags}")
add_subdirectory(rplidar_sdk)