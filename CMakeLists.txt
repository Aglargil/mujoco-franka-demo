cmake_minimum_required(VERSION 3.10)

project(mujoco-franka-demo)

set(CMAKE_CXX_STANDARD 17)

# compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OpenGL_GL_PREFERENCE GLVND)

# Eigen
find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found")
endif()

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

# Sophus
include(FetchContent)
FetchContent_Declare(
  sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG 1.24.6
)
FetchContent_MakeAvailable(sophus)

add_executable(forward_kinematics forward_kinematics.cpp)
add_executable(inverse_kinematics inverse_kinematics.cpp)
add_executable(poe poe.cpp)

target_link_libraries(forward_kinematics mujoco::mujoco glfw)
target_link_libraries(inverse_kinematics mujoco::mujoco glfw Eigen3::Eigen)
target_link_libraries(poe mujoco::mujoco glfw Eigen3::Eigen Sophus::Sophus)