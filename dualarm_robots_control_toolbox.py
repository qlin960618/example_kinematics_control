import sys
import traceback
import numpy as np
import math
import os
import time
from scipy.linalg import block_diag

import dqrobotics as dql
from dqrobotics.interfaces.vrep import DQ_VrepInterface

from nml_control_toolbox.task_control import TaskControllerSinglearm, TaskController, SystemStrategy
from nml_control_toolbox.vfi import VFIEntity, VFIRelation, VFIPrimitive, VFITargetType, VFIZoneDirection
from nml_control_toolbox.vfi import VFIManager

import helper as dqh
from vs_robot import VrepRobot

import logging

# logger init
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

CONTROL_PARAMETERS = {
    "tau": 0.004,  # sampling time
    "alpha": 0.99,  # translation over rotation weight
    "beta": 0.5,  # arm 1 over arm 2 weight
    "objb": 0.001,  # damping factor
    "n_": 40.0,  # control gain

    # VFI parameter
    "eta_d_joint": 1.0,
    "eta_d_cylinder": 1.0,
    "d_safe_cylinder": 0.0125,
    "RCM_obj_name": "VFI_RCM_Sphere",
    "eta_d_RCM": 1.0,
    "d_safe_RCM": 0.0001,
    "eta_d_p2p": 1.0,
    "d_safe_p2p": 0.1,

    "vrep_ip": "127.0.0.1",  # ip of the computer running CoppeliaSim
    "vrep_port": 20000,  # need to match remoteAPI.txt defined port

    "robot1_init_q": [0.0, 0.0, 1.57079637, 0.0, 0.61086524, 0.0],
    "robot2_init_q": [-0.61086524, 0.0, 1.57079637, 0.0, 0.0, 0.0],

    "robot_1_reference_frame_name": "VS050_reference_frame_1",
    "robot_2_reference_frame_name": "VS050_reference_frame_2#0",
    "robot_1_joints_names": [f"VS050_joint{i + 1}_1" for i in range(6)],
    "robot_2_joints_names": [f"VS050_joint{i + 1}_2#0" for i in range(6)],
    "robot_1_json": "./vs_robot/denso_vs050_DH_test.json",
    "robot_2_json": "./vs_robot/denso_vs050_DH_test.json",
    "robot_1_x": "x1",
    "robot_1_xd": "xd1",
    "robot_2_x": "x2",
    "robot_2_xd": "xd2",
}

def main(config):
    logger.info("Started Main loaded config info")
    vrep_interface = DQ_VrepInterface()
    vrep_interface.connect(config['vrep_ip'], config['vrep_port'], 100, 10)
    vrep_interface.start_simulation()

    ##################################################
    # VRep and Robot modeling initialization
    ##################################################
    robot1_interface = VrepRobot(config['robot_1_json'], vrep_interface)
    robot2_interface = VrepRobot(config['robot_2_json'], vrep_interface)
    robot1_interface.set_vrep_joint_names(config['robot_1_joints_names'])
    robot2_interface.set_vrep_joint_names(config['robot_2_joints_names'])
    robot1_interface.set_vrep_robot_ref_name(config['robot_1_reference_frame_name'])
    robot2_interface.set_vrep_robot_ref_name(config['robot_2_reference_frame_name'])
    robot1_interface.set_x_and_xd_name(config['robot_1_x'], config['robot_1_xd'])
    robot2_interface.set_x_and_xd_name(config['robot_2_x'], config['robot_2_xd'])
    robots = [robot1_interface, robot2_interface]

    # get reference frame from vrep and set it to the robot model
    for robot in robots:
        robot.apply_vrep_reference_frame()
        # Set the tooltip position, tip of the rod
        robot.set_effector(dql.DQ([1., 0., 0., 0., 0., 0., 0., 0.075]))

    robots_dim = [robot.get_dim_configuration_space() for robot in robots]
    robot1_dim = robots_dim[0]
    robot2_dim = robots_dim[1]
    # Joint limits
    robot1_q_minus = robot1_interface.get_lower_q_limit()
    robot2_q_minus = robot2_interface.get_lower_q_limit()
    robot_q_minus = [robot1_q_minus, robot2_q_minus]
    robot1_q_dot_minus = robot1_interface.get_lower_q_dot_limit()
    robot2_q_dot_minus = robot2_interface.get_lower_q_dot_limit()
    robot_q_dot_minus = [robot1_q_dot_minus, robot2_q_dot_minus]
    robot1_q_plus = robot1_interface.get_upper_q_limit()
    robot2_q_plus = robot2_interface.get_upper_q_limit()
    robot_q_plus = [robot1_q_plus, robot2_q_plus]
    robot1_q_dot_plus = robot1_interface.get_upper_q_dot_limit()
    robot2_q_dot_plus = robot2_interface.get_upper_q_dot_limit()
    robot_q_dot_plus = [robot1_q_dot_plus, robot2_q_dot_plus]

    ##################################################
    # start simulation inside try clause to ensure safe exit
    ##################################################
    try:
        logger.info("Starting simulation")
        robot1_interface.start_simulation()

        # Rate controller
        rate = dqh.RateController(loop_rate=1.0 / config['tau'], debug=False)

        robot1_q = robot1_interface.get_joint_positions()
        robot2_q = robot2_interface.get_joint_positions()
        robot_qs = [robot1_q, robot2_q]
        robot1_q_dot = np.zeros([robot1_interface.get_dim_configuration_space()])
        robot2_q_dot = np.zeros([robot2_interface.get_dim_configuration_space()])
        robot_q_dots = [robot1_q_dot, robot2_q_dot]

        ########################################################################
        # set up task controller
        ########################################################################
        single_arms = [
            TaskControllerSinglearm(
                robot=robot.robot_model,
                gain=config['n_'],
                damping=config['objb'],
                alpha=config["alpha"]
            ) for robot in robots
        ]
        task_controller = TaskController(
            system_strategy=SystemStrategy.ROBOT_PRIORITY,
            arm_controllers=single_arms,
            beta=[1 for _ in range(len(single_arms))],
        )
        task_controller.set_joint_boundary(robots_q_minus=robot_q_minus,
                                           robots_q_plus=robot_q_plus,
                                           robots_q_dot_minus=robot_q_dot_minus,
                                           robots_q_dot_plus=robot_q_dot_plus)
        task_controller.enable_joint_boundary_constraints()
        task_controller.enable_joint_boundary_velocity_constraints()

        ########################################################################
        # Example Robot Line to Robot Line VFI
        ########################################################################
        line_2_lin2_vfi = VFIEntity(
            comment="line_2_lin2_vfi",
            source_robot_id=0,
            source_joint_n=-1,
            source_tfdq_to_entity=dql.DQ([1]),
            source_prim=VFIPrimitive.LINE,
            source_prim_direction=dql.k_,
            target_type=VFITargetType.ROBOT,
            target_robot_id=1,
            target_joint_n=-1,
            target_tfdq_to_entity=dql.DQ([1]),
            target_prim=VFIPrimitive.LINE,
            target_prim_direction=dql.k_,
            d_safe=config['d_safe_cylinder'],
            eta_d=config['eta_d_cylinder'],
            vfi_direction=VFIZoneDirection.FORBIDDEN
        )

        ########################################################################
        # Example RCM  VFI
        ########################################################################
        rcm_x = vrep_interface.get_object_pose(config['RCM_obj_name'])
        rcm_vfi = VFIEntity(
            comment="rcm_vfi",
            source_robot_id=0,
            source_joint_n=-1,
            source_tfdq_to_entity=dql.DQ([1]),
            source_prim=VFIPrimitive.LINE,
            source_prim_direction=dql.k_,
            target_type=VFITargetType.STATIC,
            target_tfdq_to_entity=rcm_x,
            target_prim=VFIPrimitive.POINT,
            d_safe=config['d_safe_RCM'],
            eta_d=config['eta_d_RCM'],
            vfi_direction=VFIZoneDirection.SAFE
        )

        ########################################################################
        # Example robot Point tp Point Avoidance
        ########################################################################
        p2p_vfi = VFIEntity(
            comment="p2p_vfi",
            source_robot_id=0,
            source_joint_n=-1,
            source_tfdq_to_entity=dql.DQ([1]),
            source_prim=VFIPrimitive.POINT,
            target_type=VFITargetType.ROBOT,
            target_robot_id=1,
            target_joint_n=-1,
            target_tfdq_to_entity=dql.DQ([1]),
            target_prim=VFIPrimitive.POINT,
            d_safe=config['d_safe_p2p'],
            eta_d=config['eta_d_p2p'],
            vfi_direction=VFIZoneDirection.FORBIDDEN
        )

        ########################################################################
        # set up vfi to task controller
        ########################################################################
        vfi_manager = VFIManager()
        vfi_manager.add_entity("line2line_vfi", line_2_lin2_vfi)
        vfi_manager.add_entity("rcm_vfi", rcm_vfi)
        # vfi_manager.add_entity("p2p_vfi", p2p_vfi)
        task_controller.set_vfi_manager(vfi_manager)


        #################################################
        if "robot1_init_q" in config:
            x1_ref = robots[0].fkm(config['robot1_init_q'])
            robot1_interface.send_x_pose(x1_ref)
            robot1_interface.send_xd_pose(x1_ref)
        if "robot2_init_q" in config:
            x2_ref = robots[1].fkm(config['robot2_init_q'])
            robot2_interface.send_x_pose(x2_ref)
            robot2_interface.send_xd_pose(x2_ref)

        ##############################
        # loop initialization
        ##############################
        logger.info("Entering loop")
        rate.initialize()
        iteration = 0
        rate.sleep()
        while True:
            # Control Method
            ##################################################
            # 1, control xd from vrep
            ##################################################
            robot1_xd = robot1_interface.get_xd_pose()
            robot2_xd = robot2_interface.get_xd_pose()
            ##################################################
            # 2, control xd from trajectory
            ##################################################
            # Write your code here

            ##################################################
            for robot, q in zip(robots, robot_qs):
                robot.send_x_pose(robot.fkm(q))

            robot_qs = [robot1_q, robot2_q]
            task_controller.setup_control_setpoint_problem(
                robot_qs, [robot1_xd, robot2_xd]
            )

            try:
                # Get control signal [rad/s]
                # Solve the following quadratic program
                # Minimize     0.5x^T H x + C^T x
                #         Subject to  W. x <= w

                uq_ = task_controller.solve_control_signal()

            except ValueError as exp:
                logger.error(exp)
                uq_ = [np.zeros([robot_dim]) for robot_dim in robots_dim]

            robot1_q = robot1_q + uq_[0] * config['tau']
            robot2_q = robot2_q + uq_[1] * config['tau']

            robot1_interface.send_joint_positions(robot1_q)
            robot2_interface.send_joint_positions(robot2_q)

            ##################################################
            # End of loop maintenance
            ##################################################
            iteration += 1
            rate.sleep()
            # if iteration % 100 == 0:
            #     logger.info('Loop rate: {}'.format(rate.get_average_looprate()))


    except KeyboardInterrupt as exc:
        pass
    except Exception as exc:
        logger.error(exc)
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        traceback.print_exc()
        logger.error(str(exc_type) + " : " + str(fname) + " : " + str(exc_tb.tb_lineno))
        logger.error("{}.".format(str(exc)))

    vrep_interface.stop_simulation()
    vrep_interface.disconnect_all()
    vrep_interface.disconnect_all()


if __name__ == '__main__':
    main(CONTROL_PARAMETERS)
