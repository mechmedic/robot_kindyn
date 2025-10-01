# --------------------------------------------------------
# CKim - UR Robot with RCM control
# --------------------------------------------------------
import chun_robot
import numpy as np
import lie_group as lg


class UR_RCM_Robot(chun_robot.ChunRobot):
    def __init__(self):
        super().__init__()

        self.num_joints = 6
        
        # UR5 Kinematic parameters
        self.twist_set = np.zeros((self.num_joints,6))
        self.twist_set[0] = np.array([0,0,1, 0,0,0])
        self.twist_set[1] = np.array([0,1,0, -0.089,0,0])
        self.twist_set[2] = np.array([0,1,0, -0.089,0,0.425])
        self.twist_set[3] = np.array([0,1,0, -0.089,0,0.817])
        self.twist_set[4] = np.array([0,0,-1, -0.109,0.817,0])
        self.twist_set[5] = np.array([0,1,0, 0.006,0,0.817])

        # Initial configuration of UR5 Link csys
        self.init_tf_set = np.array([np.eye(4,4)]*(self.num_joints+1))
        self.init_tf_set[0] = np.array([ [1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1] ])
        self.init_tf_set[1] = np.array([ [1,0,0,0],[0,1,0,0],[0,0,1,0.0892],[0,0,0,1] ])
        self.init_tf_set[2] = np.array([ [0,0,1,0],[0,1,0,0.1358],[-1,0,0,0.0892],[0,0,0,1] ])
        self.init_tf_set[3] = np.array([ [0,0,1,0.4250],[0,1,0,0.0162],[ -1,0,0,0.0892],[0,0,0,1] ])
        self.init_tf_set[4] = np.array([ [-1,0,0,0.8173],[0,1,0,0.0162],[ 0,0,-1,0.0892],[0,0,0,1] ])
        self.init_tf_set[5] = np.array([ [-1,0,-0,0.8173],[0,1,0,0.1091],[ 0,0,-1,0.0892],[0,0,0,1] ])      

        # Inertia matrix of each link at the link csys
        self.inertia_set = np.array([np.eye(6,6)]*(self.num_joints))
        self.inertia_set[0] = lg.inertia(3.7, 0.0103, 0.0103, 0.0067, 0,0,0)
        self.inertia_set[1] = lg.inertia(8.393, 0.2269, 0.2269, 0.0151, 0,0,0)
        self.inertia_set[2] = lg.inertia(2.275, 0.0494, 0.0494, 0.0041, 0,0,0)
        self.inertia_set[3] = lg.inertia(1.219, 0.1112, 0.1112, 0.2194, 0,0,0)
        self.inertia_set[4] = lg.inertia(1.219, 0.1112, 0.1112, 0.2194, 0,0,0)
        self.inertia_set[5] = lg.inertia(0.1879, 0.0171, 0.0171, 0.0338, 0,0,0)

        # Initial configuration of the end effector csys. 
        self.init_tf_set[6] = np.array([ [-1,0,0,0.8173], [0,1,0,0.1091], [0,0,-1,-0.055], [0,0,0,1] ])

        # Gravitational acceleration vector in fixed space csys
        self.gravity = np.array([0,0,0, 0,0,-9.81])

        
        
        
        # RCM point in the base frame
        self.rcm_point = [0, 0, 0.1]

        # Joint limits for the robot
        self.joint_lower_limit = [-6.283185, -6.283185, -6.283185, -6.283185, -6.283185, -6.283185]
        self.joint_upper_limit = [6.283185, 6.283185, 6.283185, 6.283185, 6.283185, 6.283185]

        # Velocity limits for the robot joints
        self.joint_velocity_limit = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]