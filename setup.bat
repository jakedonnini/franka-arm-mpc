@echo off
REM Setup script for Franka Arm MPC project (Windows)
REM This script helps install dependencies and build the project

echo ==================================
echo Franka Arm MPC Setup Script
echo ==================================

REM Check for CMake
cmake --version >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo CMake not found! Please install CMake first:
    echo https://cmake.org/download/
    exit /b 1
)

echo.
echo Choose installation method:
echo 1^) vcpkg package manager
echo 2^) CMake FetchContent ^(download during build^)
echo 3^) Manual ^(use existing installations^)
echo 4^) Just build ^(skip dependency installation^)

set /p choice="Enter choice [1-4]: "

if "%choice%"=="1" goto :vcpkg
if "%choice%"=="2" goto :fetchcontent
if "%choice%"=="3" goto :manual
if "%choice%"=="4" goto :build
echo Invalid choice
exit /b 1

:vcpkg
echo Installing dependencies with vcpkg...
vcpkg --version >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo vcpkg not found! Please install vcpkg first:
    echo https://github.com/Microsoft/vcpkg
    exit /b 1
)

vcpkg install glfw3:x64-windows eigen3:x64-windows

if not defined VCPKG_ROOT (
    echo Please set VCPKG_ROOT environment variable
    exit /b 1
)

if not exist "build" mkdir build
cd build
cmake -DUSE_VCPKG=ON -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ..
goto :build_common

:fetchcontent
echo Using CMake FetchContent...
if not exist "build" mkdir build
cd build
cmake -DUSE_FETCHCONTENT=ON ..
goto :build_common

:manual
echo Using manual dependency detection...
echo Make sure MUJOCO_DIR, GLFW_DIR, and EIGEN_DIR environment variables are set if needed
if not exist "build" mkdir build
cd build
cmake ..
goto :build_common

:build
if not exist "build" mkdir build
cd build
cmake ..

:build_common
echo Building project...
cmake --build . --config Release

if %ERRORLEVEL% equ 0 (
    echo.
    echo Build complete! Executable is in build\bin\Release\
    echo.
    echo To run the simulation:
    echo   cd build\bin\Release
    echo   FrankaArmMPC.exe
    echo.
    echo Note: Make sure to update the MODEL_XML path in the source code
    echo       to point to your robot model.
) else (
    echo Build failed!
    exit /b 1
)

cd ..