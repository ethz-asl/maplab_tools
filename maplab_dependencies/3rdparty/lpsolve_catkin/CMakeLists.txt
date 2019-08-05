cmake_minimum_required(VERSION 2.8.3)
project(lpsolve_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(lpsolve_src
  URL https://github.com/ethz-asl/thirdparty_library_binaries/raw/master/lp_solve_5.5.2.0_source.tar.gz
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../lpsolve_src && pwd && cp ${PROJECT_SOURCE_DIR}/CMakeLists_lpsolve.txt ../lpsolve_src/CMakeLists.txt && cmake -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX} .
  BUILD_COMMAND cd ../lpsolve_src && make -j8
  INSTALL_COMMAND cd ../lpsolve_src && make install -j8 && pwd
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/lp_solve
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "liblpsolve*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES lpsolve)
