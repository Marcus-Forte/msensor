function (generate_proto proto_file)
# Generates the protobuf source files for a given .proto file. 
# The generates filenames  shall be named after the name of the file, followed by:
# `.pb.cc` for protobuf
#  `.grpc.pb.cc` for gRPC methods
# 
# For example, if `proto_file` is `common.proto`, the generated sources will be:
#  common.pb.cc 
#  common.grpc.pb.cc
# After they are generated, simply include them as source files to the application libraries.

if (NOT EXISTS ${proto_file})
message(FATAL_ERROR "generate_proto: '.proto' file '${proto_file}' not found.")
endif()

get_filename_component(protofile_fullpath "${proto_file}" ABSOLUTE)
get_filename_component(protofile_path "${protofile_fullpath}" PATH)
get_filename_component(protofile_name "${protofile_fullpath}" NAME_WE)

set(proto_include_args "--proto_path=${protofile_path}")
foreach(include_dir ${ARGN})
get_filename_component(proto_include_dir "${include_dir}" ABSOLUTE)
list(APPEND proto_include_args "--proto_path=${proto_include_dir}")
endforeach()

message(STATUS "Generating cpp proto files for ${protofile_fullpath}")

# Generated sources
set(proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.pb.cc")
set(proto_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/${protofile_name}.grpc.pb.cc")

add_custom_command(
      OUTPUT "${proto_srcs}" "${proto_grpc_srcs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        ${proto_include_args}
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${protofile_fullpath}"   
      DEPENDS "${protofile_fullpath}"
      )
# Have the generated header files be found.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

endfunction()