cmake_minimum_required(VERSION 2.8.3)
project(voxgraph_submap_converter)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib
  src/voxgraph-converter-node.cc
)


#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  app/voxgraph-converter-app.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

##########
# GTESTS #
##########


############
## EXPORT ##
############
cs_install()
cs_export()
