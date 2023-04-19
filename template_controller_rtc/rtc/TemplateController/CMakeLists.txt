add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  template_controller_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )
    
rtmbuild_add_library(TemplateController SHARED
  TemplateController.cpp
  TemplateControllerService_impl.cpp
  )
target_link_libraries(TemplateController
  ${catkin_LIBRARIES}
  )
set_target_properties(TemplateController PROPERTIES PREFIX "") # libTemplateController.so -> TemplateController.so                                                                           
add_dependencies(TemplateController RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl                                                                                                        

rtmbuild_add_executable(TemplateControllerComp TemplateControllerComp.cpp)
target_link_libraries(TemplateControllerComp TemplateController)
