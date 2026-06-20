cmake_minimum_required(VERSION 3.5)
project(aerial_robot_msgs)

# --- C++17 ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Build messages, services and actions ---
set(msg_files
  "msg/AerialRobotStatus.msg"
  "msg/DynamicReconfigureLevels.msg"
  "msg/FixRotor.msg"
  "msg/FlightNav.msg"
  "msg/ForceList.msg"
  "msg/FourAxisGain.msg"
  "msg/Pid.msg"
  "msg/PoseControlPid.msg"
  "msg/PosVelAcc.msg"
  "msg/PredXU.msg"
  "msg/State.msg"
  "msg/States.msg"
  "msg/TrajCoefficients.msg"
  "msg/WrenchAllocationMatrix.msg"
)

set(action_files
  "action/TrackTraj.action"
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
