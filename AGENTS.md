# AGENTS.md - AprilTag ROS Development Guide

This document provides essential guidelines for agentic coding tools (and developers) working in the `apriltag_ros` repository.

## Important: Code Review Before Commits
**Always ask the user for code review before creating any commits.** Present the changes clearly, ensure all tests and linters pass, and wait for user approval before committing to the repository.

## Project Overview

**apriltag_ros** is a ROS 2 package for AprilTag detection. It implements a C++ node that detects AprilTags in camera images and publishes their pose, ID, and metadata. Built with CMake and uses ROS 2's ament_cmake build system.

## Build, Lint, and Test Commands

### Build the Package
```bash
# Build only this package
colcon build --packages-select apriltag_ros

# Build with Address Sanitizer (ASAN) for memory debugging
colcon build --packages-select apriltag_ros --cmake-args "-DASAN=ON"

# Jetson Orin special case (avoid Eigen memcpy errors)
colcon build --packages-select apriltag_ros --cmake-args "-DCMAKE_CXX_FLAGS='${CMAKE_CXX_FLAGS} -Wno-class-memaccess'"
```

### Run Linters and Format Checks
```bash
# Run all linters (clang-format, cppcheck, ament_lint)
colcon build --packages-select apriltag_ros --cmake-args "-DCMAKE_BUILD_TYPE=Release" --cmake-target test

# Check formatting only
cd apriltag_ros && clang-format -i src/*.cpp src/*.hpp
```

### Run Tests
```bash
# Run all tests for this package
colcon build --packages-select apriltag_ros --cmake-args "-DCMAKE_BUILD_TYPE=Release" && colcon test --packages-select apriltag_ros

# Run a single test
colcon test --packages-select apriltag_ros --ctest-args -R <test_name>

# Run tests with verbose output
colcon test --packages-select apriltag_ros --ctest-args -V
```

## Code Style Guidelines

### C++ Standard and Compiler Flags
- **C++ Standard:** C++14 (set in CMakeLists.txt)
- **Strict Compilation:** All code must compile with `-Werror -Wall -Wextra -Wpedantic` flags
- **Includes:** Project headers use `.hpp` extension; system headers use standard angle brackets

### Include Order and Organization
1. ROS 2 headers (e.g., `<rclcpp/rclcpp.hpp>`)
2. Message types (e.g., `<apriltag_msgs/msg/april_tag_detection.hpp>`)
3. External libraries (e.g., `<Eigen3/Eigen/Dense>`, `<opencv2/...>`)
4. AprilTag library (e.g., `<apriltag/apriltag.h>`)
5. Project headers (e.g., `#include "tag_functions.hpp"`)

Example:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag/apriltag.h>
#include "tag_functions.hpp"
```

### Naming Conventions
- **Classes/Types:** PascalCase (e.g., `AprilTagNode`)
- **Functions/Methods:** snake_case (e.g., `detect_tags()`)
- **Variables:** snake_case (e.g., `max_hamming`, `tag_id`)
- **Constants/Macros:** UPPER_SNAKE_CASE (e.g., `IF(N, V)` macro in AprilTagNode.cpp)
- **Private Members:** Use underscore suffix or leading underscore convention

### Formatting Standards
- **Indentation:** 4 spaces (enforced by clang-format)
- **Line Length:** Keep reasonable length; clang-format will enforce
- **Braces:** Use K&R style (opening brace on same line)
- **Spacing:** Use clang-format configuration `.clang-format` (located in CMakeLists.txt reference)

### Type Safety and Usage
- **Template Usage:** Properly specialize templates for different types (e.g., `assign<T>()`, `assign_check<T>()`)
- **Atomic Types:** Support `std::atomic<T>` for thread-safe parameter assignments
- **Cv_bridge Compatibility:** Handle both old `.h` and new `.hpp` header formats:
  ```cpp
  #ifdef cv_bridge_HPP
  #include <cv_bridge/cv_bridge.hpp>
  #else
  #include <cv_bridge/cv_bridge.h>
  #endif
  ```

### Error Handling and Logging
- **ROS Logging:** Use ROS 2 logging macros (e.g., `RCLCPP_INFO`, `RCLCPP_ERROR`)
- **Parameter Validation:** Check parameter types and ranges before use
- **Null Checks:** Always check pointers before dereferencing
- **Exception Safety:** Prefer exceptions over raw error codes

### Code Organization
- **Files:** Each major component should have corresponding `.cpp` and `.hpp` files
- **Header Guards:** Use `#pragma once` or include guards
- **Composability:** Support both standalone nodes and composable components (via `rclcpp_components`)
- **Message Synchronization:** Use `message_filters::Synchronizer` for multi-topic synchronization

## Key Files and Structure
- **AprilTagNode.cpp/hpp:** Main detection node and composable component
- **tag_functions.cpp/hpp:** Tag family initialization and selection
- **pose_estimation.cpp/hpp:** Pose estimation algorithms (PnP and others)
- **conversion.cpp/hpp:** Data type conversions
- **CMakeLists.txt:** Build configuration with strict warnings enabled
- **package.xml:** ROS 2 package metadata and dependencies
- **cfg/:** Configuration files (YAML) for tag families and node parameters

## Dependencies
- **Core:** rclcpp, rclcpp_components, sensor_msgs, tf2_ros
- **Message Types:** apriltag_msgs, geometry_msgs
- **External:** apriltag, OpenCV, Eigen3, cv_bridge, image_transport
- **Testing:** ament_cmake_clang_format, ament_cmake_cppcheck, ament_lint_auto

## Important Notes
- All code must pass clang-format checks and compile without warnings
- Address Sanitizer (ASAN) can be enabled for memory issue detection
- The node supports both raw and compressed image transport
- Support both old and new versions of cv_bridge library
