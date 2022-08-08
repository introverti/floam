cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(floam)

add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CXX_FLAGS "-std=c++17 -lstdc++fs")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -pthread")
set(CMAKE_BUILD_TYPE "Debug")
set(PACKAGE_VERSION 1.0)

find_package(Eigen3)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

aux_source_directory(./src DIR_SRC)
add_executable(floam ${DIR_SRC})
target_link_libraries(floam Eigen3::Eigen ${PCL_LIBRARIES} ${CERES_LIBRARIES} stdc++fs)