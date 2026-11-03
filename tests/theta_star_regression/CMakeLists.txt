cmake_minimum_required(VERSION 3.16)
project(indoor_theta_star_regression LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
set(CMAKE_CXX_STANDARD 17)
add_executable(theta_star_regression
  regression.cpp
  ../../src/navigation2/controller/nav2_a_star_rpp/src/theta_star.cpp)
target_include_directories(theta_star_regression PRIVATE
  ../../src/navigation2/controller/nav2_a_star_rpp/include)
ament_target_dependencies(theta_star_regression nav2_costmap_2d)
target_compile_options(theta_star_regression PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
target_link_options(theta_star_regression PRIVATE -fsanitize=address,undefined)
add_executable(theta_star_replay replay.cpp
  ../../src/navigation2/controller/nav2_a_star_rpp/src/theta_star.cpp)
target_include_directories(theta_star_replay PRIVATE
  ../../src/navigation2/controller/nav2_a_star_rpp/include)
ament_target_dependencies(theta_star_replay nav2_costmap_2d)
find_package(nav2_a_star_rpp REQUIRED)
add_executable(detour_footprint_regression detour_footprint.cpp)
ament_target_dependencies(detour_footprint_regression nav2_a_star_rpp)
