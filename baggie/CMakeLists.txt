cmake_minimum_required(VERSION 3.5)
project(baggie)

if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_storage REQUIRED)

############
## Build  ##
############
pybind11_add_module(_baggie SHARED
  src/py/_baggie.cpp
  )
ament_target_dependencies(_baggie PUBLIC
  rcutils
  rosbag2_cpp
  rosbag2_compression
  rosbag2_storage
  )

##############
## Install  ##
##############
ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS _baggie
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )

install(
  PROGRAMS
    scripts/filter
    scripts/join
    scripts/split
    scripts/timeline
  DESTINATION lib/${PROJECT_NAME}/
  )

##########
## Test ##
##########
if (BUILD_TESTING)
  find_package(ament_cmake_nose REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(rclpy REQUIRED)

  ament_add_nose_test(Sequential test/test_sequential.py)
  ament_add_nose_test(SequentialCompressed test/test_sequential_compressed.py)
  ament_add_nose_test(BaggieWriter test/test_baggie_writer.py)
  ament_add_nose_test(BaggieReader test/test_baggie_reader.py)
  ament_add_nose_test(ContextManager test/test_context_mgr.py)
  ament_add_nose_test(Meta test/test_meta.py)
  ament_add_nose_test(Stamping test/test_stamping.py)
endif()

##############
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
