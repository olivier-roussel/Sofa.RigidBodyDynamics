# Sofa.RigidBodyDynamics

A plugin to handle articulated rigid bodies dynamics based on pinocchio library.

## Setup

### Install dependencies

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) >= v3.0 (e.g. `conda install pinocchio`)
- [SOFA](https://github.com/sofa-framework/sofa) >= 24.06
- [SofaPython3](https://github.com/sofa-framework/SofaPython3) >= 24.06
- [example-robot-data](https://github.com/Gepetto/example-robot-data) (e.g. `conda install example-robot-data`). Necessary if you want to run the example python scripts.

### Checkout and compile code

Checkout the repository, run `cmake`, then `make` and `make install`.

## Run examples

The plugin comes with some example python scripts to illustrate its usage.

### Set `URDF_MODEL_PATH` environment variables

The `URDF_MODEL_PATH` is used in examples scripts to set the root directory containing the URDF models. These scripts are using some URDF models provided with the [example-robot-data](https://github.com/Gepetto/example-robot-data) package. 
Your `URDF_MODEL_PATH` should be set to point to the `share` directory where you installed `example-robot-data` package. This directory should then contain a 'example-robot-data` subdirectory.
If you have installed `example-robot-data` with `conda`, this should look like this:
```
export URDF_MODEL_PATH=$CONDA_PREFIX/share
```
### Load a robot URDF to scene
```
runSofa -l SofaPython3 <SOURCE_DIR>/examples/sofa/RigidBodyDynamics/robot.py
```
You might want to check `robotWithConstraints.py` for a scene where some parts of the robot are constrainted (e.g. attached to a given position).
