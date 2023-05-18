add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  torque_auto_stabilizer_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

rtmbuild_add_library(TorqueAutoStabilizer SHARED
  TorqueAutoStabilizer.cpp
  TorqueAutoStabilizerService_impl.cpp
  )
target_link_libraries(TorqueAutoStabilizer
  ${catkin_LIBRARIES}
  )
set_target_properties(TorqueAutoStabilizer PROPERTIES PREFIX "") # libTorqueAutoStabilizer.so -> TorqueAutoStabilizer.so
add_dependencies(TorqueAutoStabilizer RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(TorqueAutoStabilizerComp TorqueAutoStabilizerComp.cpp)
target_link_libraries(TorqueAutoStabilizerComp TorqueAutoStabilizer)
