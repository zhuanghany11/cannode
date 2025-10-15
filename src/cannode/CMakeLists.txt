# CMake version
cmake_minimum_required(VERSION 3.8.2)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w -O3 -g -Wreturn-type")

project(cannode)

# CMake root display
message(STATUS "CMAKE_ROOT: " ${CMAKE_ROOT})

# Turn on the verbose
set(CMAKE_VERBOSE_MAKEFILE ON)

#add_compile_options("-w")

# module name
set(MODULE_NAME "cannode")

set(COLCON_COMPILE "on")

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

set(CMAKE_PREFIX_PATH "/opt/ros/humble" ${CMAKE_PREFIX_PATH})
link_directories(/opt/ros/humble/lib /opt/ros/humble/lib64)

# find package
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sa_msgs REQUIRED)
# find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

#set root dir
set(root_dir ${CMAKE_CURRENT_SOURCE_DIR})
# include(${root_dir}/cmake/function.cmake)
if(NOT EXISTS ${root_dir}/protobuf/out)
  make_directory(${root_dir}/protobuf/out)
endif()

if(PROTOBUF_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  message(STATUS "Protobuf include dirs: ${PROTOBUF_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries: ${PROTOBUF_LIBRARIES}")
  message(STATUS "Protobuf protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")
else()
  message(FATAL_ERROR "Protobuf not found")
endif()

# compile proto file to generate cpp file
set(proto_dir ${CMAKE_CURRENT_SOURCE_DIR}/protobuf)
file(GLOB_RECURSE MSG_PROTOS "${proto_dir}/proto/*.proto")
foreach(proto_file ${MSG_PROTOS})
  execute_process(
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    --proto_path=${proto_dir}/proto/
    --cpp_out=${proto_dir}/out/
    ${proto_file}
  )
endforeach()

# Collect all generated protobuf source files
file(GLOB_RECURSE PROTO_SRCS "${proto_dir}/out/*.pb.cc")

# Main cannode executable
file(GLOB_RECURSE
  SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
)

add_executable(${MODULE_NAME} ${SOURCES} ${PROTO_SRCS})

# Test node: send_control_cmd
add_executable(send_control_cmd 
  ${CMAKE_CURRENT_SOURCE_DIR}/test/send_control_cmd.cc
  ${PROTO_SRCS}
)

# include_sub_directories_recursively(${root_dir})


# set compile option
target_compile_options(${MODULE_NAME} PRIVATE -Wall)
target_compile_options(send_control_cmd PRIVATE -Wall)

ament_target_dependencies(${MODULE_NAME}
  PUBLIC
  rclcpp
  std_msgs
  sa_msgs
  # sensor_msgs
)

ament_target_dependencies(send_control_cmd
  PUBLIC
  rclcpp
  std_msgs
  sa_msgs
)

target_include_directories(${MODULE_NAME}
  PRIVATE
  ${OSAL_INC_DIR}
  ${SHARED_INC_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/out
  ${CMAKE_CURRENT_SOURCE_DIR}/src/low_level_controller
)

target_include_directories(send_control_cmd
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/out
)

set(ABSEL_LIBARARY
#absl::absl_check
#absl::absl_log
absl::algorithm
absl::base
absl::bind_front
absl::bits
absl::btree
absl::cleanup
absl::cord
absl::core_headers
absl::debugging
# absl::die_if_null
absl::dynamic_annotations
absl::flags
absl::flat_hash_map
absl::flat_hash_set
absl::function_ref
absl::hash
absl::layout
# absl::log_initialize
absl::log_severity
absl::memory
absl::node_hash_map
absl::node_hash_set
absl::optional
absl::span
absl::status
absl::statusor
absl::strings
absl::synchronization
absl::time
absl::type_traits
absl::utility
absl::variant
)

# message(FATAL_ERROR ${PROJ_LIBRARIES})
# link lib
target_link_libraries(
  ${MODULE_NAME}
  PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads
  ${PROTOBUF_LIBRARIES}
  ${ABSEL_LIBARARY}
)

# link lib for test node
target_link_libraries(
  send_control_cmd
  PRIVATE
  Threads::Threads
  ${PROTOBUF_LIBRARIES}
  ${ABSEL_LIBARARY}
)

install(TARGETS
  ${MODULE_NAME}
  send_control_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()








