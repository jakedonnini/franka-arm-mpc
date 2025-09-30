cmake_minimum_required(VERSION 3.20)
project(FrankaArmMPC VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Options for different dependency management approaches
option(USE_VCPKG "Use vcpkg for dependency management" OFF)
option(USE_FETCHCONTENT "Use CMake FetchContent to download dependencies" OFF)
option(DOWNLOAD_MUJOCO "Automatically download MuJoCo if not found" ON)

# ============================================================================
# Find Dependencies
# ============================================================================

# OpenGL (system library)
find_package(OpenGL REQUIRED)

# --- GLFW ---
if(USE_VCPKG)
    find_package(glfw3 REQUIRED)
    set(GLFW_TARGET glfw)
elseif(USE_FETCHCONTENT)
    include(FetchContent)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)
    set(GLFW_TARGET glfw)
else()
    # Manual GLFW search
    find_path(GLFW_INCLUDE_DIR 
        NAMES GLFW/glfw3.h
        PATHS 
            $ENV{GLFW_DIR}/include
            ${GLFW_DIR}/include
            "C:/Users/jaked/Documents/Physics_Sim/glfw-3.4/include"
            "/usr/local/include"
            "/usr/include"
        DOC "GLFW include directory"
    )
    
    find_library(GLFW_LIBRARY
        NAMES glfw3 glfw
        PATHS 
            $ENV{GLFW_DIR}/lib-vc2022
            $ENV{GLFW_DIR}/lib
            ${GLFW_DIR}/lib-vc2022
            ${GLFW_DIR}/lib
            "C:/Users/jaked/Documents/Physics_Sim/glfw-3.4/lib-vc2022"
            "/usr/local/lib"
            "/usr/lib"
        DOC "GLFW library"
    )
    
    if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
        add_library(glfw_manual UNKNOWN IMPORTED)
        set_target_properties(glfw_manual PROPERTIES
            IMPORTED_LOCATION ${GLFW_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GLFW_INCLUDE_DIR}
        )
        set(GLFW_TARGET glfw_manual)
        message(STATUS "Found GLFW: ${GLFW_LIBRARY}")
    else()
        message(FATAL_ERROR "GLFW not found. Please install GLFW or set GLFW_DIR")
    endif()
endif()

# --- Eigen ---
if(USE_VCPKG)
    find_package(Eigen3 REQUIRED)
    set(EIGEN_TARGET Eigen3::Eigen)
elseif(USE_FETCHCONTENT)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    set(EIGEN_TARGET eigen)
else()
    # Manual Eigen search
    find_path(EIGEN_INCLUDE_DIR 
        NAMES Eigen/Core
        PATHS 
            $ENV{EIGEN_DIR}
            ${EIGEN_DIR}
            "C:/Users/jaked/Documents/Physics_Sim/eigen-3.4.0"
            "/usr/local/include/eigen3"
            "/usr/include/eigen3"
        DOC "Eigen include directory"
    )
    
    if(EIGEN_INCLUDE_DIR)
        add_library(eigen_manual INTERFACE)
        target_include_directories(eigen_manual INTERFACE ${EIGEN_INCLUDE_DIR})
        set(EIGEN_TARGET eigen_manual)
        message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen not found. Please install Eigen or set EIGEN_DIR")
    endif()
endif()

# --- MuJoCo ---
# MuJoCo is more complex as it's not typically in package managers
function(download_mujoco)
    if(WIN32)
        set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-windows-x86_64.zip")
        set(MUJOCO_ARCHIVE "mujoco-windows.zip")
        set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3-windows-x86_64")
    elseif(APPLE)
        set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-macos-universal2.dmg")
        set(MUJOCO_ARCHIVE "mujoco-macos.dmg")
        set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3-macos-universal2")
    else()
        set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz")
        set(MUJOCO_ARCHIVE "mujoco-linux.tar.gz")
        set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3")
    endif()
    
    set(MUJOCO_DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/deps")
    
    if(NOT EXISTS "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_EXTRACT_DIR}")
        message(STATUS "Downloading MuJoCo...")
        file(DOWNLOAD ${MUJOCO_URL} "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
            SHOW_PROGRESS
            STATUS DOWNLOAD_STATUS
        )
        
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        if(NOT STATUS_CODE EQUAL 0)
            message(FATAL_ERROR "Failed to download MuJoCo")
        endif()
        
        message(STATUS "Extracting MuJoCo...")
        if(WIN32)
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xzf "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
                WORKING_DIRECTORY ${MUJOCO_DOWNLOAD_DIR}
            )
        else()
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xzf "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
                WORKING_DIRECTORY ${MUJOCO_DOWNLOAD_DIR}
            )
        endif()
    endif()
    
    set(MUJOCO_DIR "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_EXTRACT_DIR}" PARENT_SCOPE)
endfunction()

# Try to find MuJoCo manually first
find_path(MUJOCO_INCLUDE_DIR 
    NAMES mujoco/mujoco.h
    PATHS 
        $ENV{MUJOCO_DIR}/include
        ${MUJOCO_DIR}/include
        "C:/Users/jaked/Documents/Physics_Sim/mujoco-3.3.6-windows-x86_64/include"
        "/usr/local/include"
        "/usr/include"
    DOC "MuJoCo include directory"
)

find_library(MUJOCO_LIBRARY
    NAMES mujoco
    PATHS 
        $ENV{MUJOCO_DIR}/lib
        ${MUJOCO_DIR}/lib
        "C:/Users/jaked/Documents/Physics_Sim/mujoco-3.3.6-windows-x86_64/lib"
        "/usr/local/lib"
        "/usr/lib"
    DOC "MuJoCo library"
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    if(DOWNLOAD_MUJOCO)
        message(STATUS "MuJoCo not found, attempting to download...")
        download_mujoco()
        
        # Re-search in downloaded location
        find_path(MUJOCO_INCLUDE_DIR 
            NAMES mujoco/mujoco.h
            PATHS ${MUJOCO_DIR}/include
            NO_DEFAULT_PATH
        )
        
        find_library(MUJOCO_LIBRARY
            NAMES mujoco
            PATHS ${MUJOCO_DIR}/lib
            NO_DEFAULT_PATH
        )
    endif()
    
    if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
        message(FATAL_ERROR "MuJoCo not found. Please install MuJoCo or set MUJOCO_DIR")
    endif()
endif()

# Determine MuJoCo directory from library path for DLL copying
get_filename_component(MUJOCO_LIB_DIR ${MUJOCO_LIBRARY} DIRECTORY)
get_filename_component(MUJOCO_ROOT_DIR ${MUJOCO_LIB_DIR} DIRECTORY)

# Create MuJoCo imported target
add_library(mujoco_imported UNKNOWN IMPORTED)
set_target_properties(mujoco_imported PROPERTIES
    IMPORTED_LOCATION ${MUJOCO_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR}
)

# ============================================================================
# Create Executable
# ============================================================================

# Source files
set(SOURCES
    Franka_arm_MPC.cpp
    kinematics.cpp
    Optimizer.cpp
)

set(HEADERS
    kinematics.h
    Optimizer.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries using modern CMake
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GLFW_TARGET}
    ${EIGEN_TARGET}
    mujoco_imported
    OpenGL::GL
)

# Compiler-specific options
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE 
        /W4                     # High warning level
        /wd4127                 # Disable "conditional expression is constant" (common in Eigen)
        /wd4189                 # Disable "local variable is initialized but not referenced"
    )
    # Copy DLLs on Windows
    if(WIN32)
        # Find the MuJoCo DLL
        find_file(MUJOCO_DLL
            NAMES mujoco.dll
            PATHS 
                "${MUJOCO_ROOT_DIR}/bin"
                "${MUJOCO_ROOT_DIR}/lib"
                "${MUJOCO_LIB_DIR}"
            DOC "MuJoCo DLL"
        )
        
        if(MUJOCO_DLL)
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MUJOCO_DLL}"
                    $<TARGET_FILE_DIR:${PROJECT_NAME}>
                COMMENT "Copying MuJoCo DLL to output directory"
            )
            message(STATUS "Will copy MuJoCo DLL: ${MUJOCO_DLL}")
        else()
            message(WARNING "MuJoCo DLL not found - you may need to copy mujoco.dll manually")
        endif()
    endif()
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()

# Output directories
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ============================================================================
# Print Configuration Summary
# ============================================================================
message(STATUS "========================================")
message(STATUS "Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Use vcpkg: ${USE_VCPKG}")
message(STATUS "  Use FetchContent: ${USE_FETCHCONTENT}")
message(STATUS "  Download MuJoCo: ${DOWNLOAD_MUJOCO}")
message(STATUS "Dependencies:")
message(STATUS "  MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "  MuJoCo library: ${MUJOCO_LIBRARY}")
if(GLFW_INCLUDE_DIR)
    message(STATUS "  GLFW include: ${GLFW_INCLUDE_DIR}")
    message(STATUS "  GLFW library: ${GLFW_LIBRARY}")
endif()
if(EIGEN_INCLUDE_DIR)
    message(STATUS "  Eigen include: ${EIGEN_INCLUDE_DIR}")
endif()
message(STATUS "========================================")
