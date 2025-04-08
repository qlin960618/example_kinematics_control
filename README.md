# example_kinematics_control



## Getting started

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
1. open file `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\remoteApiConnections.txt`
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
2. Run the python script `python3 dualarm_robot_control.py`
```shell
python3 dualarm_robot_control.py
```

## Using nml_control_toolbox package

### install additional python package
unfortunately, due to some limitation with the Windows python environment, the package `nml_control_toolbox` is not 
available for Windows. Only option natively are MacOS and Linux, however, windows would still have the option of 
under WSL or WSL2
```shell
pip install --index-url https://gitea.qlin.me/api/packages/qlin/pypi/simple/ nml-control-toolbox --no-cache-dir --upgrade
```
### Runing the example
1. Open the CoppeliaSim scene `scene/VS050_TutorialControl_scene.ttt`
2. Press the play button in CoppeliaSim to start the simulation
3. Run the python script `python3 dualarm_robot_control_toolbox.py`


