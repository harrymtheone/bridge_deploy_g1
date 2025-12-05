cmake_minimum_required(VERSION 3.14)
project(bridge_g1)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bridge_core REQUIRED)
find_package(mujoco_ros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(unitree_sdk2 REQUIRED)

# Add Unitree SDK2 directly
add_subdirectory(sdk)


# Create G1 MuJoCo node
add_executable(g1_mujoco_node
    src/g1_mujoco_node.cpp
)

target_include_directories(g1_mujoco_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(g1_mujoco_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    bridge_core
    mujoco_ros_msgs
    ament_index_cpp
)

# Create G1 Real Robot node
add_executable(g1_real_node
    src/g1_real_node.cpp
)

target_include_directories(g1_real_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(g1_real_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    tf2_ros
    bridge_core
    ament_index_cpp
)

target_link_libraries(g1_real_node
    unitree_sdk2
)

# Install executable
install(TARGETS g1_mujoco_node g1_real_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install models (each model has config.yaml and model.onnx)
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

