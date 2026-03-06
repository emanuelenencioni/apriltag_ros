# AGENTS.md - apriltag_ros

Instructions for AI coding agents working on this ROS 2 C++ AprilTag detection package.

## Project Overview

ROS 2 package (ament_cmake) for detecting AprilTags in images and publishing their pose, ID, and metadata. Target platform: ROS 2 on Ubuntu 22.04. Package provides both standalone executable and composable node for efficient intraprocess communication.

## Docker Environment

**IMPORTANT**: ROS 2 and colcon are only available inside a Docker container if not installed on the host machine.

### Quick Docker Setup

The simplest approach is to mount the **current working directory** (where OpenCode is opened) as a volume:

```bash
# Get the current directory path (should be your ros2_ws or parent directory)
export WORKSPACE_PATH=$(pwd)

# Run Docker container with the current directory mounted
docker container run -it --rm --privileged \
  -v /dev/:/dev/ \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device /dev/video0 \
  -v $WORKSPACE_PATH:/home/ros2_ws \
  --net=host \
  --name="apriltag_dev" \
  --hostname="docker" \
  ros2_humble
```

**Inside the Docker container**, the workspace will be available at `/home/ros2_ws` (or the path you specified in the mount).

### Finding the Correct Workspace Location Inside Container

Once inside the container, you can verify or find the workspace:

```bash
# Check if /home/ros2_ws exists and contains the workspace
ls /home/ros2_ws

# If you need to find where the workspace was mounted, check all volumes:
mount | grep ros2

# The workspace structure should contain directories like: src/, build/, install/, log/
ls -la /home/ros2_ws/src/apriltag_ros
```

### Docker Image Source

If the Docker image is not available locally, it can be obtained from:
https://github.com/emanuelenencioni/docker_images (inside `ros_humble_desktop_full_cpu` directory)

## Build Commands (inside Docker or native ROS 2)

```bash
# Navigate to workspace (use the location where you mounted it, typically /home/ros2_ws)
cd /home/ros2_ws

# Build the package
colcon build --packages-select apriltag_ros

# Build with debug symbols
colcon build --packages-select apriltag_ros --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with AddressSanitizer enabled (for memory issue detection)
colcon build --packages-select apriltag_ros --cmake-args -DASAN=ON

# Source the workspace after building
source install/setup.bash
```

## Test & Lint Commands

```bash
# Run all tests (linting only - no unit tests currently)
colcon test --packages-select apriltag_ros

# View test results
colcon test-result --verbose

# Run clang-format check
colcon test --packages-select apriltag_ros --ctest-args -R clang_format

# Run cppcheck (static analysis)
colcon test --packages-select apriltag_ros --ctest-args -R cppcheck

# Format code with clang-format (in-place)
find src/apriltag_ros/src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

## Directory Structure

```
apriltag_ros/
├── CMakeLists.txt           # Build configuration (C++14, ament_cmake)
├── package.xml              # ROS 2 package manifest (format 3)
├── README.md                # User documentation
├── AGENTS.md                # This file
├── .clang-format            # Clang format configuration
├── cfg/
│   └── tags_36h11.yaml      # AprilTag configuration example
├── launch/
│   └── camera_36h11.launch.yml   # Example launch file with composable node
└── src/
    ├── AprilTagNode.cpp     # Main node implementation
    ├── pose_estimation.cpp  # Pose estimation methods
    ├── pose_estimation.hpp
    ├── tag_functions.cpp    # Tag family management
    ├── tag_functions.hpp
    ├── conversion.cpp       # Message conversion utilities
```

## Code Style Guidelines

### C++ Standards
- **C++ Standard**: C++14 (required in CMakeLists.txt)
- **Compiler warnings**: `-Werror -Wall -Wextra -Wpedantic` enabled (strict mode)
- **Optional**: AddressSanitizer for memory safety (enable with ASAN=ON)

### Header Guards & Pragmas
Use `#pragma once` (preferred in this codebase):
```cpp
#pragma once

#include <some_header.hpp>
```

### Naming Conventions
- **Classes/Types**: CamelCase (e.g., `AprilTagNode`, `PoseEstimation`)
- **Functions/Methods**: snake_case (e.g., `get_tag_poses`, `estimate_pose`)
- **Member variables**: snake_case with trailing underscore (e.g., `detector_`, `subscriber_`)
- **Constants**: SCREAMING_SNAKE_CASE (e.g., `MAX_HAMMING_DISTANCE`)
- **Local variables**: snake_case (e.g., `detection_count`, `tag_id`)

### Includes Order
1. Project headers (`#include "pose_estimation.hpp"`)
2. Standard library (`#include <cmath>`, `#include <string>`)
3. ROS 2 headers (`#include <rclcpp/rclcpp.hpp>`)
4. Message headers (`#include <apriltag_msgs/msg/april_tag_detection.hpp>`)
5. External libraries (`#include <apriltag/apriltag.h>`, `#include <opencv2/core.hpp>`)

### Type Usage
- Use `double` for floating-point values (not `float`)
- Use `std::string` for text
- Use `SharedPtr` for ROS message callbacks: `const sensor_msgs::msg::Image::SharedPtr msg`
- Use `rclcpp::Time` for timestamps
- Use typedefs for complex function pointers: `using pose_estimation_f = std::function<...>`
- Use `std::unordered_map` and `std::array` for collections

### ROS 2 Patterns

**Parameter Declaration:**
```cpp
this->declare_parameter<std::string>("family", "36h11");
std::string family = this->get_parameter("family").as_string();
```

**Subscriptions with Synchronizer:**
```cpp
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_sub_, info_sub_);
sync->registerCallback(&ClassName::callback, this);
```

**Logging:**
```cpp
RCLCPP_INFO(this->get_logger(), "Message: %s", value.c_str());
RCLCPP_DEBUG(this->get_logger(), "Debug: %.2f", value);
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error: %s", error_name);
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Throttled warning");
```

### Error Handling
- Check library return codes and log descriptive errors
- Use early returns on error conditions
- Gracefully handle invalid parameters
- Clean up resources in destructors
- Never throw exceptions in callbacks

```cpp
if (detection == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Failed to detect tags");
    return;
}
```

### Value Clamping & Validation
Always validate input ranges:
```cpp
hamming_distance = std::max(0, std::min(max_hamming_, hamming_distance));
```

## Launch & Runtime

### Running Standalone Node
```bash
ros2 run apriltag_ros apriltag_ros_node --ros-args \
    -r image_rect:=/camera/image \
    -r camera_info:=/camera/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```

### Running Composable Node (Preferred)
```bash
ros2 launch apriltag_ros camera_36h11.launch.yml
```

### Debugging
```bash
# Run with verbose output
ros2 run apriltag_ros apriltag_ros_node --ros-args --log-level DEBUG

# View published detections
ros2 topic echo /detections

# View TF transforms
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_frame
```

## Dependencies

### ROS 2 Packages
- `rclcpp` - C++ ROS client library
- `rclcpp_components` - Composable node support
- `sensor_msgs` - Image and CameraInfo messages
- `apriltag_msgs` - AprilTag detection messages
- `geometry_msgs` - Transform and geometry messages
- `tf2_ros` - Transform broadcaster
- `image_transport` - Efficient image transfer
- `cv_bridge` - OpenCV ↔ ROS message conversion

### External Libraries
- `apriltag` - AprilTag detection library (system dependency)
- `OpenCV` - Computer vision (core, calib3d)
- `Eigen3` - Linear algebra library

### Build & Test Dependencies
- `ament_cmake` - Build system
- `ament_lint_auto` - Automatic linting
- `ament_cmake_clang_format` - Code formatting checks
- `ament_cmake_cppcheck` - Static analysis

## Important Notes

### Jetson Orin Special Case
On Jetson Orin, build with additional flags to avoid Eigen memcpy errors:
```bash
colcon build --packages-select apriltag_ros \
  --cmake-args "-DCMAKE_CXX_FLAGS='${CMAKE_CXX_FLAGS} -Wno-class-memaccess'"
```

### Camera Intrinsics Requirements
- Camera intrinsics (matrix `P`) in `CameraInfo` are required for pose estimation
- Image timestamp and `CameraInfo` timestamp must match
- Pose is computed from the homography `H` using camera intrinsics

### AprilTag Families Supported
16h5, 25h9, 36h11, Circle21h7, Circle49h12, Custom48h12, Standard41h12, Standard52h13

### Configuration Best Practices
- Start with `max_hamming: 0` (no bit errors tolerated) and increase if tags aren't detected
- Adjust `decimate` parameter (usually 2.0) if detection is slow
- Enable `refine: 1` to snap detections to strong gradients
- Set `debug: 1` to write debugging images to the working directory

### Commits
- **ALWAYS ask the user before committing**. Do not automatically commit changes.
- Verify code passes all linting (clang-format, cppcheck) before requesting commit.
- Only commit when the user explicitly requests it or confirms correctness.

### Testing Without Camera
Use the `profile: true` parameter to enable performance profiling output to stdout.
