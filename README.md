# Franka Arm MPC Physics Simulation

![Ik_example](https://github.com/user-attachments/assets/b999048d-1a5f-4094-b9f2-b8a3d02de1d9)


A Model Predictive Control (MPC) implementation for the Franka Emika Panda robot arm using MuJoCo physics simulation.

## Features

- Real-time physics simulation of Franka Panda robot
- Forward kinematics implementation
- OpenGL visualization with GLFW
- Interactive camera controls

## Dependencies

This project requires the following libraries:

### Required Dependencies
- **MuJoCo 3.2+** - Physics simulation engine
- **GLFW 3.4+** - OpenGL context and window management  
- **Eigen 3.4.0+** - Linear algebra library
- **OpenGL** - Graphics rendering (system library)

## Quick Start

### Automated Setup (Recommended)

**Linux/macOS:**
```bash
chmod +x setup.sh
./setup.sh
```

**Windows:**
```cmd
setup.bat
```

The setup script will guide you through different installation options and automatically build the project.

### Manual Installation Options

#### Option 1: System Package Managers

**Ubuntu/Debian:**
```bash
sudo apt-get install cmake build-essential libglfw3-dev libeigen3-dev libgl1-mesa-dev
```

**macOS (with Homebrew):**
```bash
brew install cmake glfw eigen
```

**Arch Linux:**
```bash
sudo pacman -S cmake gcc glfw eigen mesa
```

#### Option 2: vcpkg (Cross-platform)
```bash
# Install vcpkg first: https://github.com/Microsoft/vcpkg
vcpkg install glfw3 eigen3

# Build with vcpkg
mkdir build && cd build
cmake -DUSE_VCPKG=ON -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake ..
cmake --build . --config Release
```

#### Option 3: CMake FetchContent (Internet Required)
```bash
mkdir build && cd build
cmake -DUSE_FETCHCONTENT=ON ..
cmake --build . --config Release
```

#### Option 4: Manual Download
```bash
# Set environment variables for dependency locations
export MUJOCO_DIR=/path/to/mujoco
export GLFW_DIR=/path/to/glfw
export EIGEN_DIR=/path/to/eigen

mkdir build && cd build
cmake ..
cmake --build . --config Release
```

## Building

### Basic Build
```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Build Options
```bash
# Use vcpkg dependencies
cmake -DUSE_VCPKG=ON -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake ..

# Download dependencies automatically  
cmake -DUSE_FETCHCONTENT=ON ..

# Disable automatic MuJoCo download
cmake -DDOWNLOAD_MUJOCO=OFF ..

# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## Usage

1. Ensure you have a Franka Panda model file (mjx_panda.xml)
2. Update the `MODEL_XML` path in `Franka_arm_MPC.cpp`
3. Run the executable:
   ```bash
   ./Franka_arm_MPC  # Linux/Mac
   ./Franka_arm_MPC.exe  # Windows
   ```

## Model Requirements

This simulation requires a MuJoCo model file for the Franka Panda robot. You can:
- Use the model from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- Create your own MJCF model file

## Project Structure

```
├── Franka_arm_MPC.cpp    # Main simulation loop
├── kinematics.cpp        # Forward kinematics implementation  
├── kinematics.h          # Kinematics header
├── CMakeLists.txt        # Build configuration
└── README.md            # This file
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Add your preferred license here]

## Troubleshooting

- **Model loading failed**: Check that `MODEL_XML` path is correct
- **GLFW initialization failed**: Ensure graphics drivers are up to date
- **Library not found**: Verify dependency paths in CMakeLists.txt
