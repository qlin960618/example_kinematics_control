import sys
import traceback
import numpy as np
import math
import os
import time
from scipy.linalg import block_diag

import dqrobotics as dql
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.utils import DQ_Geometry
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.solvers import DQ_QuadprogSolver

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
    "eta_d_RCM": 3.0,
    "d_safe_RCM": 0.0001,
    "eta_d_p2p": 1.0,
    "d_safe_p2p": 0.1,

    "vrep_ip": "127.0.0.1",  # ip of the computer running CoppeliaSim
    "vrep_port": 20000,  # need to match remoteAPI.txt defined port

    "robot1_init_q": [0.0, 0.0, 1.57079637, 0.0, 0.61086524, 0.0],
    "robot2_init_q": [-0.61086524, 0.0, 1.57079637, 0.0, 0.0, 0.0],

    "robot_1_base_name": "VS050_1",
    "robot_2_base_name": "VS050_2#0",
    "robot_1_json": "./vs_robot/denso_vs050_DH_test.json",
    "robot_2_json": "./vs_robot/denso_vs050_DH_test.json",
    "robot_1_x": "x1",
    "robot_1_xd": "xd1",
    "robot_2_x": "x2",
    "robot_2_xd": "xd2",
}


def closest_invariant_rotation_error(r, rd):
    ex_1_minus_norm = np.linalg.norm(dql.vec4(dql.conj(r) * rd - 1))
    ex_1_plus_norm = np.linalg.norm(dql.vec4(dql.conj(r) * rd + 1))
    if ex_1_plus_norm < ex_1_minus_norm:
        ex = dql.conj(r) * rd + 1
    else:
        ex = dql.conj(r) * rd - 1
    return ex


def translation_error(t, td):
    return t - td


def main(config):
    logger.info("Started Main loaded config info")
    vrep_interface = DQ_VrepInterface()
    vrep_interface.connect(config['vrep_ip'], config['vrep_port'], 100, 10)
    vrep_interface.start_simulation()
    solver = DQ_QuadprogSolver()

    ##################################################
    # VRep and Robot modeling initialization
    ##################################################
    robot1_interface = VrepRobot(config['robot_1_json'], config['robot_1_base_name'], vrep_interface)
    robot2_interface = VrepRobot(config['robot_2_json'], config['robot_2_base_name'], vrep_interface)
    robots_interface = [robot1_interface, robot2_interface]
    robot1_interface.set_x_and_xd_name(config['robot_1_x'], config['robot_1_xd'])
    robot2_interface.set_x_and_xd_name(config['robot_2_x'], config['robot_2_xd'])

    # get reference frame from vrep and set it to the robot model
    robot1_interface.apply_vrep_reference_frame()
    robot2_interface.apply_vrep_reference_frame()

    # Set the tooltip position, tip of the rod
    robot1_interface.set_effector(dql.DQ([1., 0., 0., 0., 0., 0., 0., 0.075]))
    robot2_interface.set_effector(dql.DQ([1., 0., 0., 0., 0., 0., 0., 0.075]))

    robots = [robot1_interface,
              robot2_interface]

    robot1_dim = robot1_interface.get_dim_configuration_space()
    robot2_dim = robot2_interface.get_dim_configuration_space()
    robots_dim = [robot1_dim, robot2_dim]
    # Joint limits
    robot1_q_minus = robot1_interface.get_lower_q_limit()
    robot2_q_minus = robot2_interface.get_lower_q_limit()
    robot1_q_plus = robot1_interface.get_upper_q_limit()
    robot2_q_plus = robot2_interface.get_upper_q_limit()

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

        #################################################
        x1_ref = robots[0].fkm(config['robot1_init_q'])
        x2_ref = robots[1].fkm(config['robot2_init_q'])
        robot1_interface.send_x_pose(x1_ref)
        robot2_interface.send_x_pose(x2_ref)
        robot1_interface.send_xd_pose(x1_ref)
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
            robot1_x = robot1_interface.fkm(robot1_q)
            robot2_x = robot2_interface.fkm(robot2_q)

            robot1_interface.send_x_pose(robot1_x)
            robot2_interface.send_x_pose(robot2_x)

            # Get the pose Jacobian and pose of the current tooltip pose
            Jx1 = robot1_interface.pose_jacobian(robot1_q)
            Jx2 = robot2_interface.pose_jacobian(robot2_q)

            ##################################
            # rotation Jacobian
            Jr1 = DQ_Kinematics.rotation_jacobian(Jx1)
            Jr2 = DQ_Kinematics.rotation_jacobian(Jx2)
            Nr1 = dql.haminus4(dql.rotation(robot1_xd)) @ dql.C4() @ Jr1
            Nr2 = dql.haminus4(dql.rotation(robot2_xd)) @ dql.C4() @ Jr2

            #################################
            # Translation Jacobian
            Jt1 = DQ_Kinematics.translation_jacobian(Jx1, robot1_x)
            Jt2 = DQ_Kinematics.translation_jacobian(Jx2, robot2_x)

            #################################
            # Joint limit constraints
            #################################
            robot_joint_limit = np.vstack([-np.eye(6), np.eye(6)])
            W_joint_limits = block_diag(robot_joint_limit, robot_joint_limit)

            w_joint_limits = np.hstack([-(robot1_q_minus - robot1_q),
                                        robot1_q_plus - robot1_q,
                                        -(robot2_q_minus - robot2_q),
                                        robot2_q_plus - robot2_q]) * config['eta_d_joint']

            #################################
            # Example Robot Line to Robot Line VFI
            #################################
            line1_tfdq = dql.DQ([1])
            line2_tfdq = dql.DQ([1])

            line1_x = robot1_x * line1_tfdq  # robot_pose * line_transformation
            line2_x = robot2_x * line2_tfdq  # robot_pose * line_transformation

            line1_l = dql.Ad(dql.rotation(line1_x), dql.k_)
            line2_l = dql.Ad(dql.rotation(line2_x), dql.k_)
            line1_l_dq = line1_l + dql.E_ * (dql.cross(dql.translation(line1_x), line1_l))
            line2_l_dq = line2_l + dql.E_ * (dql.cross(dql.translation(line2_x), line2_l))
            J1_l = DQ_Kinematics.line_jacobian(dql.haminus8(line1_tfdq) @ Jx1, line1_x, dql.k_)
            J2_l = DQ_Kinematics.line_jacobian(dql.haminus8(line2_tfdq) @ Jx2, line2_x, dql.k_)
            D_l12 = DQ_Geometry.line_to_line_squared_distance(line1_l_dq, line2_l_dq)
            D_l_tilda = (D_l12 - config['d_safe_cylinder']**2) * config['eta_d_cylinder']
            J_1_2 = DQ_Kinematics.line_to_line_distance_jacobian(J1_l, line1_l_dq, line2_l_dq)
            J_2_1 = DQ_Kinematics.line_to_line_distance_jacobian(J2_l, line2_l_dq, line1_l_dq)

            W_line_to_line = np.hstack([-J_1_2, -J_2_1])
            w_line_to_line = np.array([D_l_tilda])

            #################################
            # Example RCM  VFI
            #################################
            rcm_x = vrep_interface.get_object_pose(config['RCM_obj_name'])
            line1_tfdq = dql.DQ([1])

            line1_x = robot1_x * line1_tfdq  # robot_pose * line_transformation

            line1_l = dql.Ad(dql.rotation(line1_x), dql.k_)
            line1_l_dq = line1_l + dql.E_ * (dql.cross(dql.translation(line1_x), line1_l))
            J1_l = DQ_Kinematics.line_jacobian(dql.haminus8(line1_tfdq) @ Jx1, line1_x, dql.k_)
            D_rcm = DQ_Geometry.point_to_line_squared_distance(dql.translation(rcm_x), line1_l_dq)
            D_rcm_tilda = (D_rcm - config['d_safe_RCM']**2) * config['eta_d_RCM']
            J_1_rcm = DQ_Kinematics.line_to_point_distance_jacobian(J1_l, line1_l_dq, dql.translation(rcm_x))

            W_rcm = np.hstack([J_1_rcm, np.zeros([1, robot2_dim])])
            w_rcm = np.array([-D_rcm_tilda])


            #################################
            # Example Rpbpt Point tp Point Avoidence
            #################################
            p1_tfdq = dql.DQ([1])
            p2_tfdq = dql.DQ([1])
            eta_d_p2p = config['eta_d_p2p']
            d_safe_p2p = config['d_safe_p2p']

            p1_x = robot1_x * p1_tfdq
            p2_x = robot2_x * p2_tfdq
            D_p2p = DQ_Geometry.point_to_point_squared_distance(dql.translation(p1_x), dql.translation(p2_x))
            D_p2p_tilda = (D_p2p - d_safe_p2p**2) * eta_d_p2p
            J_p1_2 = DQ_Kinematics.point_to_point_distance_jacobian(
                DQ_Kinematics.translation_jacobian(dql.haminus8(p1_tfdq) @ Jx1, p1_x), dql.translation(p1_x),
                dql.translation(p2_x)
            )
            J_p2_1 = DQ_Kinematics.point_to_point_distance_jacobian(
                DQ_Kinematics.translation_jacobian(dql.haminus8(p2_tfdq) @ Jx2, p2_x), dql.translation(p2_x),
                dql.translation(p1_x)
            )

            W_point_to_point = np.hstack([J_p1_2, J_p2_1])
            w_point_to_point = np.array([-D_p2p_tilda])



            #################################
            # Quadratic Programing
            #################################
            robot1_t = dql.translation(robot1_x)
            robot2_t = dql.translation(robot2_x)
            robot1_r = dql.rotation(robot1_x)
            robot2_r = dql.rotation(robot2_x)

            alpha = config['alpha']
            beta = config['beta']
            objb = config['objb']

            err_t1 = translation_error(robot1_t, dql.translation(robot1_xd)).vec4()
            err_t2 = translation_error(robot2_t, dql.translation(robot2_xd)).vec4()
            err_r1 = closest_invariant_rotation_error(robot1_r, dql.rotation(robot1_xd)).vec4()
            err_r2 = closest_invariant_rotation_error(robot2_r, dql.rotation(robot2_xd)).vec4()

            H1_objb = objb * np.eye(robot1_dim)
            H2_objb = objb * np.eye(robot2_dim)
            H1 = (alpha * Jt1.T @ Jt1 + (1.0 - alpha) * Nr1.T @ Nr1) * beta + H1_objb
            H2 = (alpha * Jt2.T @ Jt2 + (1.0 - alpha) * Nr2.T @ Nr2) * (1.0 - beta) + H2_objb
            H = 2 * block_diag(H1, H2)
            f1 = 2 * (alpha * err_t1.T @ Jt1 + (1.0 - alpha) * err_r1.T @ Nr1) * beta
            f2 = 2 * (alpha * err_t2.T @ Jt2 + (1.0 - alpha) * err_r2.T @ Nr2) * (1.0 - beta)
            f = np.hstack([f1, f2])

            # W_ineq = np.zeros([1, robot1_dim + robot2_dim])
            # w_ineq = np.zeros([1])
            W_ineq = np.vstack([
                W_joint_limits,
                W_line_to_line,
                W_rcm,
                # W_point_to_point
            ])
            w_ineq = np.hstack([
                w_joint_limits,
                w_line_to_line,
                w_rcm,
                # w_point_to_point
            ])

            try:
                # Get control signal [rad/s]
                # Solve the following quadratic program
                # Minimize     0.5x^T H x + C^T x
                #         Subject to  W. x <= w

                uq_vec = solver.solve_quadratic_program(
                    H, f, W_ineq, w_ineq, None, None,
                )

                uq_ = [uq_vec[0:robot1_dim], uq_vec[robot1_dim:]]

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
