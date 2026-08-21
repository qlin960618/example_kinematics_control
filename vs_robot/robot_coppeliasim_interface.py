from .robot_loader import RobotLoader


class CoppeliaSimRobot:
    """Connect a DQ robot model to named objects in a CoppeliaSim scene."""

    def __init__(self, config_path, coppeliasim):
        self.robot_model = RobotLoader(config_path).get_kinematics()
        self.coppeliasim = coppeliasim

        self.robot_reference_frame_name = None
        self.joint_names = None
        self.robot_reference_frame_dq = None
        self.x_name = None
        self.xd_name = None

    def set_joint_names(self, joint_names):
        assert len(joint_names) == self.robot_model.get_dim_configuration_space()
        self.joint_names = joint_names

    def set_reference_frame_name(self, reference_frame_name):
        self.robot_reference_frame_name = reference_frame_name

    ##################################################
    # Initialization related
    ##################################################
    def apply_reference_frame(self):
        self.robot_reference_frame_dq = self.coppeliasim.get_object_pose(
            self.robot_reference_frame_name
        )
        self.set_reference_frame(self.robot_reference_frame_dq)

    def set_x_and_xd_name(self, x_name, xd_name):
        self.x_name = x_name
        self.xd_name = xd_name

    def get_reference_frame(self):
        return self.coppeliasim.get_object_pose(self.robot_reference_frame_name)

    ##################################################
    # Robot modeling related
    ##################################################
    def fkm(self, qs=None):
        if qs is None:
            return self.robot_model.fkm(self.get_joint_positions())
        return self.robot_model.fkm(qs)

    def __getattr__(self, item):
        return getattr(self.robot_model, item)

    ##################################################
    # Quick tools
    ##################################################
    def send_xd_pose(self, xd_):
        assert self.xd_name is not None, "xd_name is not set"
        return self.coppeliasim.set_object_pose(self.xd_name, xd_)

    def send_x_pose(self, x_):
        assert self.x_name is not None, "x_name is not set"
        return self.coppeliasim.set_object_pose(self.x_name, x_)

    def get_xd_pose(self):
        assert self.xd_name is not None, "xd_name is not set"
        return self.coppeliasim.get_object_pose(self.xd_name)

    ##################################################
    # Robot driver interface
    ##################################################
    def get_joint_positions(self):
        assert self.joint_names is not None
        return self.coppeliasim.get_joint_positions(self.joint_names)

    def send_joint_positions(self, qs, update_pose=False):
        assert self.joint_names is not None
        self.coppeliasim.set_joint_positions(self.joint_names, qs)

        if self.x_name is not None and update_pose:
            self.coppeliasim.set_object_pose(self.x_name, self.robot_model.fkm(qs))
