# Sofa.RigidBodyDynamics

A plugin to handle articulated rigid bodies dynamics based on [pinocchio](https://github.com/stack-of-tasks/pinocchio) library.

## Installation

Clone the repository first:
```
git clone https://github.com/olivier-roussel/Sofa.RigidBodyDynamics.git
```
and change your current directory to the checkouted folder.

### Build with Pixi (recommanded)

To build Sofa.RigidBodyDynamics from source the easiest way is to use [Pixi](https://pixi.sh/).
Pixi is a cross-platform package management tool for developers that will install all required dependencies in `.pixi` directory. If you don't have Pixi installed yet, you can get it [here](https://pixi.sh/latest/#installation).

Run the following command to install dependencies, configure and build the project:
```
pixi run build
```
The project will be built in the build directory. 

Alternatively, you can run `pixi shell` and build the project with `cmake` manually. This command will activate an environment in your shell with all dependencies already installed.

Then, try the plugin with one of the provided examples throught the following Pixi tasks.

#### example-load-robot

Creates a simple SOFA scene and loads an URDF model (here a Franka Emika Panda robot) inside it:
```
pixi run -e example-load-robot example-robot-data/robots/panda_description/urdf/panda.urdf
```
You can replace the loaded URDF file at your convenience. The [example-robot-data](https://github.com/Gepetto/example-robot-data) installed package provides some URDF file, for example the PAL Robotics Talos humanoid:
```
pixi run -e example-load-robot example-robot-data/robots/talos_data/robots/talos_reduced_corrected.urdf
```
Or the Open Dynamic Robot Initiative Solo quadruped robot:
```
pixi run -e example-load-robot example-robot-data/robots/solo_description/robots/solo.urdf
```

#### example-simple-arm

Creates a simple SOFA scene and loads an URDF model of a simple 1-dof manipulator with an inverse control towards an effector position, using the [SoftRobots.Inverse plugin](https://github.com/SofaDefrost/SoftRobots.Inverse).

```
pixi run -e example-simple-arm
```

### Build from source

Build and install with `cmake`:
```
cmake -S . -B build
cmake --build build/ --config Release --target install
```

#### Dependencies
- CMake
- a C++-20 compliant compiler
- [pinocchio](https://github.com/stack-of-tasks/pinocchio) >= v3.0
- [SOFA](https://github.com/sofa-framework/sofa) >= 24.12
- [SOFA plugin SofaPython3](https://github.com/sofa-framework/SofaPython3) >= 24.12

Install with `conda`:
```
conda install cmake cxx-compiler ninja pinocchio sofa-framework::sofa-devel sofa-framework::sofa-python3
```

#### Optional dependencies (for examples)
- [example-robot-data](https://github.com/Gepetto/example-robot-data)
- [SOFA plugin SoftRobots](https://github.com/SofaDefrost/SoftRobots) >= 24.12
- [SOFA plugin SoftRobots.Inverse](https://github.com/SofaDefrost/SoftRobots.Inverse) >= 24.12

Install with `conda`:
```
conda install example-robot-data sofa-framework::sofa-app sofa-framework::sofa-softrobotsinverse
```

#### Set `URDF_MODEL_PATH` environment variables

The `URDF_MODEL_PATH` is used in examples scripts to set the root directory containing the URDF models. These scripts are using some URDF models provided with the [example-robot-data](https://github.com/Gepetto/example-robot-data) package. 
Your `URDF_MODEL_PATH` should be set to point to the `share` directory where you installed `example-robot-data` package. This directory should then contain a 'example-robot-data` subdirectory.
If you have installed `example-robot-data` with `conda`, this should look like this:
```
export URDF_MODEL_PATH=$CONDA_PREFIX/share
```
#### Load a robot URDF to scene
```
runSofa -l SofaPython3 <SOURCE_DIR>/examples/sofa/RigidBodyDynamics/robot.py <URDF_FILE>
```
You might want to check `robotWithConstraints.py` for a scene where some parts of the robot are constrainted (e.g. attached to a given position).
