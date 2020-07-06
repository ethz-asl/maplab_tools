cmake_minimum_required(VERSION 2.8.3)
project(image_debayer)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra -O3 -DNDEBUG")

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}_lib
  src/image-reconstruction.cc
)

#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  app/image-debayer-node.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

cs_install()
cs_export()
