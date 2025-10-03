cmake_minimum_required(VERSION 3.5)

project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
  #add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

option(ASAN "use AddressSanitizer to detect memory issues" OFF)

if(ASAN)
    set(ASAN_FLAGS "\
        -fsanitize=address \
        -fsanitize=bool \
        -fsanitize=bounds \
        -fsanitize=enum \
        -fsanitize=float-cast-overflow \
        -fsanitize=float-divide-by-zero \
        -fsanitize=nonnull-attribute \
        -fsanitize=returns-nonnull-attribute \
        -fsanitize=signed-integer-overflow \
        -fsanitize=undefined \
        -fsanitize=vla-bound \
        -fno-sanitize=alignment \
        -fsanitize=leak \
        -fsanitize=object-size \
    ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(geometry_msgs REQUIRED)
find_package(apriltag REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS} ${APRILTAG_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
if(cv_bridge_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(cv_bridge_HPP)
endif()

add_executable(${PROJECT_NAME}_node src/AprilTagNode.cpp
src/conversion.cpp
src/pose_estimation.cpp
src/tag_functions.cpp
)

target_link_libraries(${PROJECT_NAME}_node apriltag)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    apriltag_msgs
    tf2_ros
    image_transport
    cv_bridge
    Eigen3
    OpenCV
    apriltag
)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
