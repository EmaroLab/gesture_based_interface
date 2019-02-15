#!/usr/bin/env python
###########
# IMPORTS #
###########
import rospy
import baxter_interface
from baxter_pykdl import baxter_kinematics
import numpy as np
from scipy import optimize

####################
# GLOBAL VARIABLES #
####################
WEIGHT_GRIPPER_POSITION = 2 
WEIGHT_JTS_TRAVEL = 0.001

class NewLimbMover:
    def __init__(self, limb):
        self.limb = limb    #just a string
        self.interface = baxter_interface.Limb(limb)
        self.kinem = baxter_kinematics(limb)
        self.jt_names = self.interface.joint_names()
        self.curr_jts = self.get_curr_joints()
        self.target_jts = self.curr_jts
        self.target_jts_dict = self.interface.joint_angles()
        self.target_pos = np.array([0., 0., 0.])    #just for initializing

    def get_curr_joints(self):
        '''Returns current joint angles as ndarray.'''

        jt_dict = self.interface.joint_angles()
        jt_array = np.zeros(len(self.jt_names))
        for i, name in enumerate(self.jt_names):
            jt_array[i] = jt_dict[name]
        return jt_array

    def iksolve(self, ptarget): 
        '''Given ndarray of target end-effector position [x,y,z] in Baxter's base frame,
        update ndarray as well as dict of target joint angles.'''

        self.target_pos = []
        self.target_pos.append(ptarget.x)
        self.target_pos.append(ptarget.y)
        self.target_pos.append(ptarget.z)
        
        self.curr_jts = self.get_curr_joints()
        
        self.target_jts, success = self.find_best_jts()
        self.target_jts_dict = dict(zip(self.jt_names, self.target_jts))

        # rospy.loginfo("message: %s", message)
        if success:
            return True
        else:
            return False

    def find_best_jts(self):
        '''Solves for optimal joint values (first 6 joints), returns ndarray of all joint angles
        (first 6 optimal, last=0).'''

        # rospy.loginfo("ptarget: %s", self.target_pos)
        jt_bounds = [(-1.7 ,1.7),(-2.14, 1.04),(-3.05, 3.05),(-0.05, 2.61),(-3.05, 3.05),(-1.57, 2.09)]
        # jt_bounds = np.array([(-1.7 ,1.7),(-2.14, 1.04),(-3.05, 3.05),(-0.05, 2.61),(-3.05, 3.05),(-1.57, 2.09)])
        solver_options = {"maxiter":5000,"disp":False}
        result = optimize.minimize(self.calc_error, self.curr_jts[:-1], jac=self.calc_jac, bounds=jt_bounds, options=solver_options)
        jts = result.x
        jts = np.hstack((jts, 0))
        success = result.success
        # message = result.message
        return jts, success
        
    def calc_error(self, jt_values):
        '''Given ndarray of first 6 joint angles, returns a weighted error objective function.'''

        fk_translation = self.fksolver_trans(np.hstack((jt_values,0)))
        error_gripper_position = (np.linalg.norm(self.target_pos - fk_translation))**2
        error_jts_travel = (np.linalg.norm(jt_values - self.curr_jts[:-1]))**2
        
        total_error = WEIGHT_GRIPPER_POSITION*error_gripper_position + WEIGHT_JTS_TRAVEL*error_jts_travel
        return total_error

    def fksolver_trans(self, jt_angles):
        '''Given ndarray of all 7 joint angles, calculates translation from base to end-effector'''
        
        jt_dict = dict(zip(self.jt_names, jt_angles))
        trans_rot = self.kinem.forward_position_kinematics(joint_values=jt_dict)
        trans = trans_rot[:3]
        # rospy.loginfo("fk_trans: %s", trans)
        return trans

    def calc_jac(self, jt_values):
        '''Given ndarray of first 6 joint angles, returns Jacobian of the error objective function.'''
        
        jac = np.zeros(6)

        all_jts = np.hstack((jt_values,0))
        jt_dict = dict(zip(self.jt_names, all_jts))

        FK = self.fksolver_trans(all_jts)
        JM = self.kinem.jacobian(joint_values=jt_dict)

        for i, x in enumerate(jt_values):
            dfdxi2 = 2*WEIGHT_JTS_TRAVEL*(x - self.curr_jts[i])
            diff0 = FK[0] - self.target_pos[0]
            diff1 = FK[1] - self.target_pos[1]
            diff2 = FK[2] - self.target_pos[2]
            dfdxi1 = 2*WEIGHT_GRIPPER_POSITION*(diff0*JM[0,i] + diff1*JM[1,i] + diff2*JM[2,i])

            jac[i] = dfdxi1 + dfdxi2

        return jac


    def move(self, des_joint_vels):
        '''Sets desired joint velocities'''

        self.interface.set_joint_velocities(des_joint_vels)
