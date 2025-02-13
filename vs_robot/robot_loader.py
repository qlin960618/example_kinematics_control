import numpy as np
import math
import os
import logging

# logger init
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

import dqrobotics as dql
from dqrobotics.interfaces.json11 import DQ_JsonReader
from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulator, DQ_SerialManipulatorDH

import json


# Class definition of VS050 DENSO robot for pathology

class RobotLoader:
    def __init__(self, json_path=None):
        # Standard of VS050

        if json_path is None or not os.path.isfile(json_path):
            raise ValueError("robot.json not specified")

        try:
            with open(json_path) as j_f:
                jdata = json.load(j_f)

        except Exception as e:
            raise ValueError("DH loading file read error")

        reader = DQ_JsonReader()

        if jdata['robot_type'] == "DQ_SerialManipulatorDH":
            self.robot = reader.get_serial_manipulator_dh_from_json(json_path)
        elif jdata['robot_type'] == "DQ_SerialManipulatorDenso":
            # self.robot = reader.get_serial_manipulator_denso_from_json(json_path)
            raise ValueError("Not implemented yet")
        else:
            raise ValueError("json parameter type definition error: " + str(type))

    def get_kinematics(self):
        return self.robot


#############################for testing only################################
def main():
    jpath = "./denso_vs050_DH_test.json"
    # jpath="./denso_vs050_denso_11U483.json"
    robot = RobotLoader(jpath)
    logger.info(str(robot))
    logger.info(str(robot.fkm([0, 0, 0, 0, 0, 0])))


if __name__ == '__main__':
    main()
#############################for testing only################################
