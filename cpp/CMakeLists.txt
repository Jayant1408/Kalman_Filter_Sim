cmake_minimum_required(VERSION 3.14)
project(kalman_filter_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(kf STATIC src/kf.cpp)
target_include_directories(kf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kf PUBLIC Eigen3::Eigen)

add_executable(kf_demo src/main.cpp)
target_link_libraries(kf_demo PRIVATE kf)
