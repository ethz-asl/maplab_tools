cmake_minimum_required(VERSION 2.8.3)
project(semantic_cloud_publisher)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


############
## EXPORT ##
############
cs_install()

catkin_python_setup()

cs_export()
