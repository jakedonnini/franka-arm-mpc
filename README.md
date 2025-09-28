# Franka Arm MPC Physics Simulation

A Model Predictive Control (MPC) implementation for the Franka Emika Panda robot arm using MuJoCo physics simulation.

## Features

- Real-time physics simulation of Franka Panda robot
- Forward kinematics implementation
- OpenGL visualization with GLFW
- Interactive camera controls

## Dependencies

This project requires the following libraries:

### Required Dependencies
- **MuJoCo 3.3.6+** - Physics simulation engine
- **GLFW 3.4+** - OpenGL context and window management  
- **Eigen 3.4.0+** - Linear algebra library
- **OpenGL** - Graphics rendering (system library)

### Installation

#### Option 1: Manual Installation (Current Setup)
1. Download and extract dependencies to your preferred location:
   - [MuJoCo](https://github.com/google-deepmind/mujoco/releases) 
   - [GLFW](https://www.glfw.org/download.html)
   - [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

2. Update the paths in `CMakeLists.txt`:
   ```cmake
   set(MUJOCO_DIR "path/to/mujoco")
   set(GLFW_DIR "path/to/glfw") 
   set(EIGEN_DIR "path/to/eigen")
   ```

#### Option 2: Using vcpkg (Recommended)
```bash
vcpkg install glfw3 eigen3
# Note: MuJoCo needs to be installed manually
```

#### Option 3: Using Conan
```bash
conan install . --build=missing
```

## Building

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
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