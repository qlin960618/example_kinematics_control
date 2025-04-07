# example_kinematics_control



## Getting started

### Python requirements:
- python3
  - dqrobotics
  - quadprog
  - numpy
  - scipy
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


