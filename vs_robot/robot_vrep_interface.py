from .robot_loader import RobotLoader
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_SerialManipulator
import dqrobotics as dql


class VrepRobot:
    def __init__(self, config_path, vrep_robot_name, vrep):
        self.robot_model = RobotLoader(config_path).get_kinematics()
        if vrep_robot_name.find('_') >= 0:
            self.vrep_name_prefix, self.vrep_name_suffix = vrep_robot_name.split('_')
            self.vrep_name_suffix = '_' + self.vrep_name_suffix
            self.vrep_name_prefix = self.vrep_name_prefix + "_"
        else:
            self.vrep_name_prefix = vrep_robot_name + "_"
            self.vrep_name_suffix = ""

        self.vrep = vrep

        self.vrep_robot_ref_name = None
        self.vrep_robot_joint_names = None
        self.vrep_robot_ref_frame_dq = None
        self.vrep_x_name = None
        self.vrep_xd_name = None

        self._process_vrep_robot_obj_name()

    ##################################################
    # initialization related
    ##################################################
    def apply_vrep_reference_frame(self):
        self.vrep_robot_ref_frame_dq = self.vrep.get_object_pose(self.vrep_robot_ref_name)
        self.set_reference_frame(self.vrep_robot_ref_frame_dq)

    def set_x_and_xd_name(self, x_name, xd_name):
        self.vrep_x_name = x_name
        self.vrep_xd_name = xd_name

    def get_reference_frame_from_vrep(self):
        return self.vrep.get_object_pose(self.vrep_robot_ref_name)

    def _process_vrep_robot_obj_name(self):
        # print(base_name)
        n_dims = self.get_dim_configuration_space()
        self.vrep_robot_ref_name = self.vrep_name_prefix + "reference_frame" + self.vrep_name_suffix
        self.vrep_robot_joint_names = []
        for i in range(n_dims):
            self.vrep_robot_joint_names.append(
                self.vrep_name_prefix + "joint{:d}".format(i + 1) + self.vrep_name_suffix)

    ##################################################
    # robot modeling related
    ##################################################
    def fkm(self, qs=None):
        if qs is None:
            return self.robot_model.fkm(self.get_joint_positions())
        else:
            return self.robot_model.fkm(qs)

    def __getattr__(self, item):
        return getattr(self.robot_model, item)

    ##################################################
    # quick tools
    ##################################################
    def send_xd_pose(self, xd_):
        assert self.vrep_xd_name is not None, "xd_name is not set"
        return self.vrep.set_object_pose(self.vrep_xd_name, xd_)

    def send_x_pose(self, x_):
        assert self.vrep_x_name is not None, "x_name is not set"
        return self.vrep.set_object_pose(self.vrep_x_name, x_)

    def get_xd_pose(self):
        assert self.vrep_xd_name is not None, "xd_name is not set"
        return self.vrep.get_object_pose(self.vrep_xd_name)

    def _check_sim_running(self):
        assert self.vrep.is_simulation_running(), "WARNING: Trying to set joints when simulation is not running"

    ##################################################
    # Overloading function for Robot Driver Interface
    ##################################################
    def get_joint_positions(self):
        assert self.vrep_robot_joint_names is not None
        return self.vrep.get_joint_positions(self.vrep_robot_joint_names)

    def send_joint_positions(self, qs, force=False, update_pose=False):
        assert self.vrep_robot_joint_names is not None
        if not force:
            self._check_sim_running()

        ret = self.vrep.set_joint_positions(self.vrep_robot_joint_names, qs)

        if self.vrep_x_name is not None and update_pose:
            x_ = self.robot_model.fkm(qs)
            ret = ret and self.vrep.set_object_pose(self.vrep_x_name, x_)
        return ret

    ##################################################
    # Vrep Interface related
    ##################################################

    def stop_simulation(self, *nargs, **kwargs):
        self.vrep.stop_simulation(*nargs, **kwargs)

    def start_simulation(self, *nargs, **kwargs):
        self.vrep.start_simulation(*nargs, **kwargs)

    def disconnect_all(self, *nargs, **kwargs):
        self.vrep.disconnect_all(*nargs, **kwargs)
