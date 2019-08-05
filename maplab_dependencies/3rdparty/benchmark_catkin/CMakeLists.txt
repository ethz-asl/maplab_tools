cmake_minimum_required(VERSION 2.8.3)
project(benchmark_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
  benchmark_src
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG 336bb8db986cc52cdf0cefa0a7378b9567d1afee # 1.3.0
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../benchmark_src &&
     cmake .
       -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
       -DBUILD_SHARED_LIBS:BOOL=true
       -DCMAKE_BUILD_TYPE:STRING=Release
  BUILD_COMMAND cd ../benchmark_src && make -j 8
  INSTALL_COMMAND cd ../benchmark_src && $(MAKE) install -j 8
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/benchmark
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libbenchmark*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES benchmark
          CFG_EXTRAS add-benchmark.cmake)
