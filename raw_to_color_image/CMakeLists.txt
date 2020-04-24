cmake_minimum_required(VERSION 2.8.3)
project(raw_to_color_image)

find_package(catkin_simple REQUIRED)
catkin_simple()

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}_lib
  src/r2c-manager.cc
  src/ros-utils.cc
)

#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  app/raw-to-color-node.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

cs_install()
cs_export()
