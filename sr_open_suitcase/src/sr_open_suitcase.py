#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest("sr_open_suitcase")
import rospy

from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PlaceGoal, PlaceAction
from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest, GraspPlanningResponse
from arm_navigation_msgs.srv import GetMotionPlanRequest, GetMotionPlanResponse, FilterJointTrajectory, FilterJointTrajectoryRequest
from arm_navigation_msgs.msg import DisplayTrajectory, JointLimits
from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from trajectory_msgs.msg import JointTrajectory
from sr_utilities.srv import getJointState
from sr_utilities.srv import getJointState
from visualization_msgs.msg import Marker
from tf import transformations
import actionlib

from sr_pick_and_place.execution import Execution

import math, sys
import numpy as np

from sr_open_suitcase.srv import OpenSuitcase, OpenSuitcaseResponse

import time
import copy

ARM_NAMES = ['ShoulderJRotate', 'ShoulderJSwing', 'ElbowJSwing', 'ElbowJRotate', "WRJ1", "WRJ2"]

class SrOpenSuitcase(object):
    """
    """

    #The xyz distance we want the palm link to be from the grasping point
    DISTANCE_TO_GRASP = [-0.05, 0.0, 0.04]
    #The distance used when computing the approach
    APPROACH_DISTANCE = 0.05

    def __init__(self, ):
        """
        """
        self.markers_pub_ = rospy.Publisher("~markers", Marker)
        self.execution = Execution(use_database = False)

        self.suitcase_src_ = rospy.Service("~open_suitcase", OpenSuitcase, self.open_lid)

    def open_lid(self, suitcase_req):
        suitcase = suitcase_req.suitcase
        self.display_suitcase_(suitcase)

        semi_circle = self.go_to_mechanism_and_grasp_(suitcase)

        if semi_circle == None:
            rospy.logerr("Failed to approach.")
            sys.exit(1)

        time.sleep(1.0)

        self.lift_lid_(suitcase, semi_circle)

        #release the lid
        self.execution.grasp_release_exec()

    def go_to_mechanism_and_grasp_(self, suitcase):
        #compute the full trajectory
        semi_circle = self.compute_semi_circle_traj_(suitcase)

        #compute the pregrasp and grasp
        grasp = Grasp()
        grasp_pose_ = PoseStamped()

        #the grasp is the first item of the semi circle
        grasp_pose_ = semi_circle[0]

        # copy the grasp_pose as a pre-grasp_pose
        pre_grasp_pose_ = copy.deepcopy(grasp_pose_)

        # add desired_approach_distance along the approach vector. above the object to plan pre-grasp pose
        # TODO: use the suitcase axis to approach from the perpendicular
        pre_grasp_pose_.pose.position.x = pre_grasp_pose_.pose.position.x - self.APPROACH_DISTANCE

        #TODO: find better postures
        grasp.pre_grasp_posture.name = [ "FFJ0", "FFJ3", "FFJ4", "LFJ0", "LFJ3", "LFJ4", "LFJ5", "MFJ0", "MFJ3", "MFJ4", "RFJ0", "RFJ3", "RFJ4", "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"]
        grasp.pre_grasp_posture.position = [0.0]*18
        grasp.pre_grasp_posture.position[ grasp.pre_grasp_posture.name.index("THJ4") ] = 58.0
        grasp.pre_grasp_posture.position[ grasp.pre_grasp_posture.name.index("THJ5") ] = -50.0

        grasp.grasp_posture.name = [ "FFJ0", "FFJ3", "FFJ4", "LFJ0", "LFJ3", "LFJ4", "LFJ5", "MFJ0", "MFJ3", "MFJ4", "RFJ0", "RFJ3", "RFJ4", "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"]
        grasp.grasp_posture.position = [0.0]*18
        grasp.grasp_posture.position[ grasp.grasp_posture.name.index("THJ1") ] = 87.0
        grasp.grasp_posture.position[ grasp.grasp_posture.name.index("THJ2") ] = 30.0
        grasp.grasp_posture.position[ grasp.grasp_posture.name.index("THJ3") ] = -15.0
        grasp.grasp_posture.position[ grasp.grasp_posture.name.index("THJ4") ] = 58.0
        grasp.grasp_posture.position[ grasp.grasp_posture.name.index("THJ5") ] = 40.0

        grasp.grasp_pose = grasp_pose_.pose

        # for distance from 0 (grasp_pose) to desired_approach distance (pre_grasp_pose) test IK/Collision and save result
        # decompose this in X steps depending on distance to do and max speed
        motion_plan_res=GetMotionPlanResponse()
        interpolated_motion_plan_res = self.execution.plan.get_interpolated_ik_motion_plan(pre_grasp_pose_, grasp_pose_, False)

        # check the result (depending on number of steps etc...)
        if (interpolated_motion_plan_res.error_code.val == interpolated_motion_plan_res.error_code.SUCCESS):
            number_of_interpolated_steps=0
            # check if one approach trajectory is feasible
            for interpolation_index, traj_error_code in enumerate(interpolated_motion_plan_res.trajectory_error_codes):
                if traj_error_code.val!=1:
                    rospy.logerr("One unfeasible approach-phase step found at "+str(interpolation_index)+ "with val " + str(traj_error_code.val))
                    break
                else:
                    number_of_interpolated_steps=interpolation_index

            # if trajectory is feasible then plan reach motion to pre-grasp pose
            if number_of_interpolated_steps+1==len(interpolated_motion_plan_res.trajectory.joint_trajectory.points):
                rospy.loginfo("Grasp number approach is possible, checking motion plan to pre-grasp")
                #print interpolated_motion_plan_res

                # check and plan motion to this pre_grasp_pose
                motion_plan_res = self.execution.plan.plan_arm_motion( "right_arm", "jointspace", pre_grasp_pose_ )

        # execution part
        if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
            #put hand in pre-grasp posture
            if self.execution.pre_grasp_exec(grasp)<0:
                rospy.logerr("Failed to go in pregrasp.")
                sys.exit()

            #go there
            # filter the trajectory
            filtered_traj = self.execution.filter_traj_(motion_plan_res)

            self.execution.display_traj_( filtered_traj )

            # reach pregrasp pose
            if self.execution.send_traj_( filtered_traj )<0:
                time.sleep(20) # TODO use actionlib here

            # approach
            if self.execution.send_traj_( interpolated_motion_plan_res.trajectory.joint_trajectory )<0:
                rospy.logerr("Failed to approach.")
                sys.exit()
            time.sleep(20) # TODO use actionlib here

            #grasp
            if self.execution.grasp_exec(grasp)<0:
                rospy.logerr("Failed to grasp.")
                sys.exit()
            time.sleep(20) # TODO use actionlib here

        else:
            #Failed, don't return the computed traj
            return None

        #return the full traj
        return semi_circle


    def lift_lid_(self, suitcase, semi_circle):
        while len(semi_circle) > 0:
            self.execution.plan_and_execute_step_(semi_circle)
            time.sleep(0.5)

        return OpenSuitcaseResponse(OpenSuitcaseResponse.SUCCESS)

    def compute_semi_circle_traj_(self, suitcase, nb_steps = 50):
        poses = []

        #compute a semi-circular trajectory, starting
        # from the suitcase mechanism, rotating
        # around the suitcase axis
        target = PoseStamped()
        target.header.frame_id = suitcase.header.frame_id
        target.pose = suitcase.opening_mechanism.pose_stamped.pose

        #axis_x and axis_z are the projection of the mechanism model
        # onto the suitcase axis (point around which we want to rotate)
        mechanism = [suitcase.opening_mechanism.pose_stamped.pose.position.x,
                     suitcase.opening_mechanism.pose_stamped.pose.position.y,
                     suitcase.opening_mechanism.pose_stamped.pose.position.z]
        axis = [suitcase.lid_axis_a.x + (suitcase.lid_axis_b.x - suitcase.lid_axis_a.x) / 2.0,
                suitcase.lid_axis_a.y + (suitcase.lid_axis_b.y - suitcase.lid_axis_a.y) / 2.0,
                suitcase.lid_axis_a.z + (suitcase.lid_axis_b.z - suitcase.lid_axis_a.z) / 2.0]

        #We're always starting with the palm straight along the x axis
        #TODO: use real suitcase axis instead?
        suitcase_axis = (1, 0, 0)
        for i in range(0, nb_steps + 1):
            #we're rotating from this angle around the suitcase axis to point towards the suitcase
            rotation_angle = float(i) * math.pi / 2.0 / float(nb_steps)

            ####
            # POSITION
            target.pose.position.x = axis[0] - ((axis[0] - mechanism[0] - self.DISTANCE_TO_GRASP[0]) * math.cos(rotation_angle))
            target.pose.position.z = axis[2] + ((axis[0] - mechanism[0] - self.DISTANCE_TO_GRASP[0]) * math.sin(rotation_angle)) + self.DISTANCE_TO_GRASP[2]

            ####
            # ORIENTATION
            #limit the wrj1 axis
            rotation_angle = min(rotation_angle, math.radians(15.) )
            # add 90 degrees to point the axis toward the suitcase
            rotation_angle += math.pi / 2.0
            #orientation, palm z axis pointing towards the suitcase axes
            #z toward suitcase
            toward_z = transformations.quaternion_about_axis(math.pi / 2.0, (0,0,1))
            #then rotate as we go up to continue pointing at the axis
            step_rotation = transformations.quaternion_about_axis( rotation_angle, suitcase_axis)
            #combine those transform:
            orientation = transformations.quaternion_multiply(toward_z, step_rotation)

            target.pose.orientation.x = orientation[0]
            target.pose.orientation.y = orientation[1]
            target.pose.orientation.z = orientation[2]
            target.pose.orientation.w = orientation[3]
            poses.append( copy.deepcopy(target) )

        return poses

    def display_suitcase_(self, suitcase):
        #display axes
        axes_marker = Marker()
        axes_marker.header.frame_id = suitcase.header.frame_id
        axes_marker.ns = "suitcase"
        axes_marker.id = 0
        axes_marker.type = Marker.LINE_STRIP
        axes_marker.action = Marker.ADD
        axes_marker.points.append(suitcase.lid_axis_a)
        axes_marker.points.append(suitcase.lid_axis_b)
        axes_marker.scale.x = 0.02

        axes_marker.color.a = 0.4;
        axes_marker.color.r = 0.1;
        axes_marker.color.g = 0.47;
        axes_marker.color.b = 0.88;

        self.markers_pub_.publish( axes_marker )

        #display mechanism
        mechanism_marker = Marker()
        mechanism_marker.header.frame_id = suitcase.header.frame_id
        mechanism_marker.ns = "suitcase"
        mechanism_marker.id = 1
        mechanism_marker.type = Marker.CUBE
        mechanism_marker.action = Marker.ADD

        mechanism_marker.pose = suitcase.opening_mechanism.pose_stamped.pose
        mechanism_marker.scale = suitcase.opening_mechanism.dimensions

        mechanism_marker.color.a = 0.4;
        mechanism_marker.color.r = 0.1;
        mechanism_marker.color.g = 0.47;
        mechanism_marker.color.b = 0.88;

        self.markers_pub_.publish( mechanism_marker )

if __name__ =="__main__":
    rospy.init_node("execution")
    execute = SrOpenSuitcase()

    detected_suitcase = OpenSuitcase()

    rospy.spin()


