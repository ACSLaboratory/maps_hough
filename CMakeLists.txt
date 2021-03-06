cmake_minimum_required(VERSION 3.5)
project(maps_hough)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(hough_test src/test.cpp src/grid_map.cpp src/io.cpp src/common.cpp src/hough.cpp src/manipulatemap.cpp)
ament_target_dependencies(hough_test rclcpp ament_index_cpp OpenCV)

install(TARGETS
  hough_test
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  datasets
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
