# Example Kinematics Control

## Getting Started

This tutorial is designed for those who have completed the [MATLAB lesson](https://github.com/dqrobotics/learning-dqrobotics-in-matlab) and would like to dive deeper into optimization-based control using DQRobotics in Python.

### Python Requirements

* Python 3.10–3.13

  * dqrobotics
  * dqrobotics-interface-coppeliasim-zmq
  * quadprog
  * numpy
  * scipy
  * venv

**Before we start, let’s create a virtual environment to ensure we isolate our changes locally:**

```shell
python3 -m venv venv
source venv/bin/activate # or venv\Scripts\activate.bat for Windows
```

```shell
python3 -m pip install --upgrade --pre \
    dqrobotics dqrobotics-interface-coppeliasim-zmq quadprog numpy scipy
```

### CoppeliaSim Requirements

* CoppeliaSim 4.4 or newer; 4.6 or newer is recommended.
* The built-in **ZMQ Remote API Server** add-on must be running.

1. Download CoppeliaSim from [Coppelia Robotics](https://www.coppeliarobotics.com/).
2. Run the installation procedure.
3. Start CoppeliaSim and confirm that **Modules → Connectivity → ZMQ Remote API Server** is running. It normally starts automatically and listens on port `23000`.

The controllers connect to `localhost:23000` by default. To use another host or port, change `coppeliasim_host` or `coppeliasim_port` in the relevant script. Do not edit `remoteApiConnections.txt`: that file configures the deprecated legacy remote API, not ZeroMQ.

The new interface class is `DQ_CoppeliaSimInterfaceZMQ`, supplied by the `dqrobotics-interface-coppeliasim-zmq` package.

## Running the Tutorial

1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`.
2. Ensure the ZMQ Remote API Server add-on is running. The script starts and stops the simulation remotely.
3. Run the Python script:

```shell
python3 dualarm_robots_control.py
```

4. Select the `xd1` and `xd2` in the scene, move them around, and observe how the robot arm follows the end effector.

### Learning Tasks

1. Add a Robot Point to Robot Point Constraint in the code. Define the point at the tip of the cylinder with your desired radius. *Try both safe and forbidden zone directions.*
2. Add a Robot Point to Static Plane Constraint to the code and the robot in the scene. Explore different options by modifying BOTH the scene and the code.
3. Add a third robot to the scene and try to control all three robots simultaneously.

## Using the nml\_control\_toolbox Package

### Installing Additional Python Packages

Due to limitations in the Windows Python environment, the package `nml_control_toolbox` is not available for Windows. It is only natively supported on macOS and Linux. Windows users may try WSL or WSL2 (this has not been tested yet).

```shell
pip install --index-url https://gitea.qlin.me/api/packages/qlin/pypi/simple/ nml-control-toolbox --no-cache-dir --upgrade
```

### Running the Example

1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`.
2. Ensure the ZMQ Remote API Server add-on is running. The script starts and stops the simulation remotely.
3. Run the Python script:

```shell
python3 dualarm_robot_control_toolbox.py
```

4. Select `xd1` and `xd2` in the scene, move them around, and observe how the robot arm follows the end effector.

### Learning Tasks

1. Add a Robot Point to Robot Point Constraint in the code. Define the point at the tip of the cylinder with your desired radius. *Try both safe and forbidden zone directions.*
2. Add a Robot Point to Static Plane Constraint to the code and the robot in the scene. Explore different options by modifying BOTH the scene and the code.
3. Add a third robot to the scene and try to control all three robots simultaneously.
