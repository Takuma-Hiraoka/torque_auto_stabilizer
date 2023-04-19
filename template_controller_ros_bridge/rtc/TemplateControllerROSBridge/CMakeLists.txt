add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  template_controller_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(TemplateControllerROSBridge SHARED TemplateControllerROSBridge.cpp)
target_link_libraries(TemplateControllerROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES})
set_target_properties(TemplateControllerROSBridge PROPERTIES PREFIX "")

add_executable(TemplateControllerROSBridgeComp TemplateControllerROSBridgeComp.cpp)
target_link_libraries(TemplateControllerROSBridgeComp TemplateControllerROSBridge)

install(TARGETS TemplateControllerROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS TemplateControllerROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
