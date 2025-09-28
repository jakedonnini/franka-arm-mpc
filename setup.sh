#!/bin/bash
# Setup script for Franka Arm MPC project
# This script helps install dependencies and build the project

set -e  # Exit on any error

echo "=================================="
echo "Franka Arm MPC Setup Script"
echo "=================================="

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    OS="windows"
else
    echo "Unsupported OS: $OSTYPE"
    exit 1
fi

echo "Detected OS: $OS"

# Function to install dependencies with package managers
install_with_vcpkg() {
    echo "Installing dependencies with vcpkg..."
    
    if ! command -v vcpkg &> /dev/null; then
        echo "vcpkg not found. Please install vcpkg first:"
        echo "https://github.com/Microsoft/vcpkg"
        exit 1
    fi
    
    vcpkg install glfw3 eigen3
    
    echo "To build with vcpkg, use:"
    echo "cmake -DUSE_VCPKG=ON -DCMAKE_TOOLCHAIN_FILE=\$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake .."
}

install_system_packages() {
    case $OS in
        "linux")
            echo "Installing system packages for Linux..."
            if command -v apt-get &> /dev/null; then
                sudo apt-get update
                sudo apt-get install -y cmake build-essential libglfw3-dev libeigen3-dev libgl1-mesa-dev
            elif command -v yum &> /dev/null; then
                sudo yum install -y cmake gcc-c++ glfw-devel eigen3-devel mesa-libGL-devel
            elif command -v pacman &> /dev/null; then
                sudo pacman -S cmake gcc glfw eigen mesa
            else
                echo "Package manager not supported. Please install dependencies manually."
                exit 1
            fi
            ;;
        "macos")
            echo "Installing system packages for macOS..."
            if command -v brew &> /dev/null; then
                brew install cmake glfw eigen
            else
                echo "Homebrew not found. Please install Homebrew first:"
                echo "https://brew.sh/"
                exit 1
            fi
            ;;
        "windows")
            echo "For Windows, we recommend using vcpkg or manual installation"
            echo "See README.md for detailed instructions"
            ;;
    esac
}

build_project() {
    echo "Building project..."
    
    if [ ! -d "build" ]; then
        mkdir build
    fi
    
    cd build
    
    # Choose build method based on available tools
    if [ "$1" == "vcpkg" ] && command -v vcpkg &> /dev/null; then
        cmake -DUSE_VCPKG=ON -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" ..
    elif [ "$1" == "fetchcontent" ]; then
        cmake -DUSE_FETCHCONTENT=ON ..
    else
        cmake ..
    fi
    
    cmake --build . --config Release
    
    echo "Build complete! Executable is in build/bin/"
    cd ..
}

# Main menu
echo ""
echo "Choose installation method:"
echo "1) System packages (recommended for Linux/macOS)"
echo "2) vcpkg package manager"
echo "3) CMake FetchContent (download during build)"
echo "4) Manual (use existing installations)"
echo "5) Just build (skip dependency installation)"

read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        install_system_packages
        build_project
        ;;
    2)
        install_with_vcpkg
        build_project "vcpkg"
        ;;
    3)
        echo "Using CMake FetchContent..."
        build_project "fetchcontent"
        ;;
    4)
        echo "Using manual dependency detection..."
        echo "Make sure MUJOCO_DIR, GLFW_DIR, and EIGEN_DIR environment variables are set if needed"
        build_project
        ;;
    5)
        build_project
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "Setup complete!"
echo ""
echo "To run the simulation:"
echo "  cd build/bin"
echo "  ./FrankaArmMPC"
echo ""
echo "Note: Make sure to update the MODEL_XML path in the source code to point to your robot model."