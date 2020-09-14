cmake_minimum_required(VERSION 3.13)
project(samson)

set(CMAKE_CXX_STANDARD 20)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(autodiff REQUIRED)

add_executable(samson src/iiwa.cpp)

target_include_directories(samson PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)

target_link_libraries (samson PUBLIC Eigen3::Eigen autodiff::autodiff)