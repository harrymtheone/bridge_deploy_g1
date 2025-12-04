cmake_minimum_required(VERSION 3.14)
project(g1_description)

find_package(ament_cmake REQUIRED)

# Install MJCF models
install(DIRECTORY mjcf/
    DESTINATION share/${PROJECT_NAME}/mjcf
)

# Install meshes
install(DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/mjcf/meshes
)

ament_package()

