cmake_minimum_required(VERSION 3.0.2)
project(torque_auto_stabilizer_rtc)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  )

rtmbuild_init()

catkin_package(
  LIBRARIES TorqueAutoStabilizerServiceSkel TorqueAutoStabilizerServiceStub
  )

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

add_subdirectory(rtc)
