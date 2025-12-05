cmake_minimum_required(VERSION 3.14)
project(g1_description)

find_package(ament_cmake REQUIRED)

# Install MJCF models
install(DIRECTORY mjcf/
    DESTINATION share/${PROJECT_NAME}/mjcf
)

# Install meshes
install(DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
)

# Install xacro files
install(DIRECTORY xacro/
    DESTINATION share/${PROJECT_NAME}/xacro
)

# Install URDF files
install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()

