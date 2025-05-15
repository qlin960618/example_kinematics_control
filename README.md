# example_kinematics_control

## Getting started

This tutorial is designed for those has completed the [MATLAB lesson](https://github.com/dqrobotics/learning-dqrobotics-in-matlab) 
and would like to have a deep dive into optimization-based control using DQRobotics in Python. 

### Python requirements:
- python3
  - dqrobotics
  - quadprog
  - numpy
  - scipy
  - venv

**Before we start, lets create a virtual environment just to make sure we isolate out change locally**
```shell
python3 -m venv venv
source venv/bin/activate # or venv\Scripts\activate.bat for windows
````
```shell
python3 -m pip install dqrobotics quadprog numpy scipy
```
### CoppeliaSim requirements:
- CoppeliaSim edu, (at the time of writing this, 4.4.0 and 4.7.0 are supported)

1. Download CoppeliaSim from [Coppelia Robotics](https://www.coppeliarobotics.com/previousVersions)
2. Run installation procedure

### open additional port for vrep connection
Windows: 
1. open file 
   - Windows: `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\remoteApiConnections.txt`
   - Linux: `[CoppeliaSim Installed location]/CoppeliaSimEdu/remoteApiConnections.txt`
   - MacOS: `[CoppeliaSim.app -> Open Package Contents]/Contents/Resources/remoteApiConnections.txt`
2. add the following line to the file: (replace with desired ports)
```
portIndex2_port             = 19998
portIndex2_debug            = false
portIndex2_syncSimTrigger   = true

portIndex3_port             = 19999
portIndex3_debug            = false
portIndex3_syncSimTrigger   = true
```

## Running the tutorial
1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`
2. Press the play button in CoppeliaSim to start the simulation
3. Run the python script `python3 dualarm_robot_control.py`
    ```shell
    python3 dualarm_robot_control.py
    ```
4. Selected the `xd1` and `xd2` in the scene move them around and see how the robot arm follows the end effector.

### Learning Task
1. Add Robot Point to Robot Point Constraint into the code, Lets defined the point to be at the tip od the cylinder 
and with radius of you're desired. *try out both safe and forbidden zone direction*
2. Add a Robot point to statics plane constraint to the code and robot in the scene, and try out the different options. For 
this you will have to modify BOTH the scene and the code. 
3. Add a third robots into the scene and see if you can control all three robots at the same time.

## Using nml_control_toolbox package

### install additional python package
unfortunately, due to some limitation with the Windows python environment, the package `nml_control_toolbox` is not 
available for Windows. Only option natively are MacOS and Linux, however, windows would still have the option of 
under WSL or WSL2 (This is not tested yet).
```shell
pip install --index-url https://gitea.qlin.me/api/packages/qlin/pypi/simple/ nml-control-toolbox --no-cache-dir --upgrade
```
### Runing the example
1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`
2. Press the play button in CoppeliaSim to start the simulation
3. Run the python script `python3 dualarm_robot_control_toolbox.py`
4. Selected the `xd1` and `xd2` in the scene move them around and see how the robot arm follows the end effector.

### Learning Task
1. Add Robot Point to Robot Point Constraint into the code, Lets defined the point to be at the tip od the cylinder 
and with radius of you're desired. *try out both safe and forbidden zone direction*
2. Add a Robot point to statics plane constraint to the code and robot in the scene, and try out the different options. For 
this you will have to modify BOTH the scene and the code. 
3. Add a third robots into the scene and see if you can control all three robots at the same time.

