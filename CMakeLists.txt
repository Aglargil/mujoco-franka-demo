cmake_minimum_required(VERSION 3.10)

project(mujoco-franka-demo)

set(CMAKE_CXX_STANDARD 17)

# compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OpenGL_GL_PREFERENCE GLVND)

# Eigen
find_package(Eigen3 REQUIRED)

# Mujoco
find_package(mujoco REQUIRED)
if(NOT mujoco_FOUND)
    message(FATAL_ERROR "Mujoco not found")
endif()

# GLFW
find_package(glfw3 REQUIRED)
if(NOT glfw3_FOUND)
    message(FATAL_ERROR "GLFW not found")
endif()

add_executable(forward_kinematics forward_kinematics.cpp)
add_executable(inverse_kinematics inverse_kinematics.cpp)

target_link_libraries(forward_kinematics mujoco::mujoco glfw)
target_link_libraries(inverse_kinematics mujoco::mujoco glfw Eigen3::Eigen)
