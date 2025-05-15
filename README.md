# Example Kinematics Control

## Getting Started

This tutorial is designed for those who have completed the [MATLAB lesson](https://github.com/dqrobotics/learning-dqrobotics-in-matlab) and would like to dive deeper into optimization-based control using DQRobotics in Python.

### Python Requirements

* Python 3

  * dqrobotics
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
python3 -m pip install dqrobotics quadprog numpy scipy
```

### CoppeliaSim Requirements

* CoppeliaSim EDU (at the time of writing this, versions 4.4.0 and 4.7.0 are supported)

1. Download CoppeliaSim from [Coppelia Robotics](https://www.coppeliarobotics.com/previousVersions).
2. Run the installation procedure.

### Opening Additional Ports for VREP Connection

#### Windows

1. Open the file located at:

   * Windows: `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\remoteApiConnections.txt`
   * Linux: `[CoppeliaSim Installation Location]/CoppeliaSimEdu/remoteApiConnections.txt`
   * MacOS: `[CoppeliaSim.app -> Open Package Contents]/Contents/Resources/remoteApiConnections.txt`

2. Add the following lines to the file (replace with your desired ports):

```plaintext
portIndex2_port             = 19998
portIndex2_debug            = false
portIndex2_syncSimTrigger   = true

portIndex3_port             = 19999
portIndex3_debug            = false
portIndex3_syncSimTrigger   = true
```

## Running the Tutorial

1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`.
2. Press the play button in CoppeliaSim to start the simulation.
3. Run the Python script:

```shell
python3 dualarm_robot_control.py
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
2. Press the play button in CoppeliaSim to start the simulation.
3. Run the Python script:

```shell
python3 dualarm_robot_control_toolbox.py
```

4. Select `xd1` and `xd2` in the scene, move them around, and observe how the robot arm follows the end effector.

### Learning Tasks

1. Add a Robot Point to Robot Point Constraint in the code. Define the point at the tip of the cylinder with your desired radius. *Try both safe and forbidden zone directions.*
2. Add a Robot Point to Static Plane Constraint to the code and the robot in the scene. Explore different options by modifying BOTH the scene and the code.
3. Add a third robot to the scene and try to control all three robots simultaneously.
