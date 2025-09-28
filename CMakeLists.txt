cmake_minimum_required(VERSION 3.20)
project(FrankaArmMPC)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Paths ----
set(MUJOCO_DIR "C:/Users/jaked/Documents/Physics_Sim/mujoco-3.3.6-windows-x86_64")
set(GLFW_DIR "C:/Users/jaked/Documents/Physics_Sim/glfw-3.4")
set(EIGEN_DIR "C:/Users/jaked/Documents/Physics_Sim/eigen-3.4.0")  # adjust path

# ---- Include directories ----
include_directories(
    ${MUJOCO_DIR}/include
    ${GLFW_DIR}/include
    ${EIGEN_DIR}
    ${CMAKE_SOURCE_DIR}  # for your own headers
)

# ---- Source files ----
set(SOURCES
    Franka_arm_MPC.cpp
    kinematics.cpp
)

# ---- Add executable ----
add_executable(Franka_arm_MPC ${SOURCES})

# ---- Link libraries ----
target_link_libraries(Franka_arm_MPC
    ${MUJOCO_DIR}/lib/mujoco.lib
    ${GLFW_DIR}/lib-vc2022/glfw3.lib
    opengl32
)
