#!/usr/bin/python
import rospy
import numpy as np
import PyKDL as kdl
import kdl_parser_py.urdf as parse
import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF
import random
from hexapod_msgs.srv import GetIKSolver
import sys

NUM_LEGS = 6

class LegKinematics(object):
    def __init__(self, ):
        rospy.init_node("leg_kinematics_node")
        
        rospy.loginfo("Starting LegKinematics as leg_kinematics_node.")
        self.root_name = ""
        self.tip_name = ""
        kdl_tree=None
        def replace_none(x, v):
            if x is None:
                return v
            return x
        
        def joint_list_to_kdl(q):
            if q is None:
                return None
            if type(q) == np.matrix and q.shape[1] == 0:
                q = q.T.tolist()[0]
            q_kdl = kdl.JntArray(len(q))
            for i, q_i in enumerate(q):
                q_kdl[i] = q_i
            return q_kdl

        urdf = URDF.from_parameter_server()
        if not urdf:
            rospy.logerr("Unable to load urdf_description")
            return
        
        link_suffix = ["r1","r2","r3","l1","l2","l3"]
        base_link = urdf.get_root()
        end_link_name = "leg_tip_"

        if kdl_tree is None:
            success, kdl_tree = parse.treeFromUrdfModel(urdf)
        self.tree = kdl_tree
        self.urdf = urdf
        self.chain = list()
        for i in range(0,NUM_LEGS):
            end_link = end_link_name + link_suffix[i]
            self.chain.append(kdl_tree.getChain(base_link, end_link))
        # print(end_link)
        # print(self.chain)

        self.joint_limits_lower = list()
        self.joint_limits_upper = list()
        self.joint_types = list()
        for jnt_name in self.get_joint_names():
            # print(jnt_name)
            jnt = urdf.joint_map[jnt_name]
            if jnt.limit is not None:
                self.joint_limits_lower.append(jnt.limit.lower)
                self.joint_limits_upper.append(jnt.limit.upper)
            else:
                self.joint_limits_lower.append(None)
                self.joint_limits_upper.append(None)
            self.joint_types.append(jnt.joint_type)

        mins_kdl = joint_list_to_kdl(self.joint_limits_lower)
        maxs_kdl = joint_list_to_kdl(self.joint_limits_upper)
        # print(self.joint_limits_lower)
        

        
        
        self.joint_limits_lower = np.array([replace_none(jl, -np.inf)
                                            for jl in self.joint_limits_lower])
        self.joint_limits_upper = np.array([replace_none(jl, np.inf)
                                            for jl in self.joint_limits_upper])


        self.joint_types = np.array(self.joint_types)
        self.num_joints = len(self.get_joint_names())
        self._fk_kdl = list()
        self._ik_v_kdl = list()
        self._ik_p_kdl = list()
        self._jac_kdl = list()
        self._dyn_kdl = list()  
        maxiter=100

        for i in range(0,NUM_LEGS):    
            self._fk_kdl.append(kdl.ChainFkSolverPos_recursive(self.chain[i]))
            self._ik_v_kdl.append(kdl.ChainIkSolverVel_pinv(self.chain[i]))
            self._ik_p_kdl.append(kdl.ChainIkSolverPos_NR_JL(self.chain[i],mins_kdl, maxs_kdl, self._fk_kdl[i], self._ik_v_kdl[i], maxiter, sys.float_info.epsilon))
            # self._jac_kdl.append(kdl.ChainJntToJacSolver(self.chain[i]))
            # self._dyn_kdl.append(kdl.ChainDynParam(self.chain[i], kdl.Vector.Zero()))
        
        srv_server_handler = rospy.Service("get_ik", GetIKSolver, self.ik_service_callback)


    def ik_service_callback(self,req):

        for i in range(len(req.leg_number)):
            print(i)
            leg_dest_pos = req.target_position[i]
            jnt_pos_in = kdl.JntArray(NUM_LEGS)
            jnt_pos_out = kdl.JntArray(NUM_LEGS)

            for j in range(0,NUM_LEGS):
                jnt_pos_in[i] = req.current_joints[i].joint[j]
            
            frame_kdl = kdl.Frame(kdl.Vector(leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z))
            ik_solver = self._ik_p_kdl[req.leg_number[i]]

            if np.any(q_guess == None):
                # use the midpoint of the joint limits as the guess
                lower_lim = np.where(np.isfinite(self.joint_limits_lower), self.joint_limits_lower, 0.)
                upper_lim = np.where(np.isfinite(self.joint_limits_upper), self.joint_limits_upper, 0.)
                q_guess = (lower_lim + upper_lim) / 2.0
                q_guess = np.where(np.isnan(q_guess), [0.]*len(q_guess), q_guess)
            
            

            ik_valid = ik_solver(self.chain[i],)

    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain("base_link", "leg_tip_r1",
                                   links=links, fixed=fixed)

    def get_joint_limits(self):
        return self.joint_limits_lower, self.joint_limits_upper



if __name__ == "__main__":
    name_node = LegKinematics()
    rospy.spin()