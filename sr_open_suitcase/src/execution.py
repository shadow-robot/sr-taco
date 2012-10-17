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
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sr_utilities.srv import getJointState
from planification import Planification
from sr_utilities.srv import getJointState

from sr_open_suitcase.srv import OpenSuitcase, OpenSuitcaseResponse

import time
import copy

ARM_NAMES = ['ShoulderJRotate', 'ShoulderJSwing', 'ElbowJSwing', 'ElbowJRotate', "WRJ1", "WRJ2"]

class Execution(object):
    """
    """

    def __init__(self, ):
        """
        """
        #initialize the planner
        self.plan = Planification()

        self.display_traj_pub_ = rospy.Publisher("/joint_path_display", DisplayTrajectory, latch=True)
        self.send_traj_pub_ = rospy.Publisher("/command", JointTrajectory, latch=True)

        rospy.loginfo("Waiting for services  /getJointState, /trajectory_filter_unnormalizer/filter_trajectory, /database_grasp_planning")
        rospy.wait_for_service("/getJointState")
        rospy.wait_for_service("/trajectory_filter_unnormalizer/filter_trajectory")

        rospy.loginfo("  OK services found")

        self.get_joint_state_ = rospy.ServiceProxy("/getJointState", getJointState)
        self.trajectory_filter_ = rospy.ServiceProxy("/trajectory_filter_unnormalizer/filter_trajectory", FilterJointTrajectory)

        self.suitcase_src_ = rospy.Service("~open_suitcase", OpenSuitcase, self.open_lid)

    def open_lid(self, suitcase_req):
        motion_plan_res=GetMotionPlanResponse()

        #TODO: compute targets based on mechanism pose and lid axes
        target_pose_ = PoseStamped()
        target_pose_.header.frame_id = "/world";
        target_pose_.pose.position.x = 0.63
        target_pose_.pose.position.y = 0.0
        target_pose_.pose.position.z = 1.3

        target_pose_.pose.orientation.x = 0.375
        target_pose_.pose.orientation.y = 0.155
        target_pose_.pose.orientation.z = 0.844
        target_pose_.pose.orientation.w = 0.351

        next_target_pose_ = copy.deepcopy(target_pose_)
        next_target_pose_.pose.position.z = next_target_pose_.pose.position.z + 0.05
        next_target_pose_.pose.position.x = next_target_pose_.pose.position.x + 0.05

        interpolated_motion_plan_res = self.plan.get_interpolated_ik_motion_plan(target_pose_, next_target_pose_, False, num_steps = 1, frame="/world")

        # check the result (depending on number of steps etc...)
        if (interpolated_motion_plan_res.error_code.val == interpolated_motion_plan_res.error_code.SUCCESS):
            number_of_interpolated_steps=0
            for interpolation_index, traj_error_code in enumerate(interpolated_motion_plan_res.trajectory_error_codes):
                if traj_error_code.val!=1:
                    rospy.logerr("One unfeasible approach-phase step found at "+str(interpolation_index)+ " with val " + str(traj_error_code.val))
                else:
                    number_of_interpolated_steps=interpolation_index

        if number_of_interpolated_steps+1==len(interpolated_motion_plan_res.trajectory.joint_trajectory.points):
            motion_plan_res = self.plan.plan_arm_motion( "right_arm", "jointspace", target_pose_ )

            if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
                rospy.loginfo("OK, motion planned, executing it.")

        #at the lid, compute lifting of the lid
        if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
            #go there
            # filter the trajectory
            filtered_traj = self.filter_traj_(motion_plan_res)

            self.display_traj_( filtered_traj )
            self.send_traj_( filtered_traj )

            #approach
            #time.sleep(15)
            self.send_traj_( interpolated_motion_plan_res.trajectory.joint_trajectory )

        else:
            rospy.logerr("Lifting impossible")
            return OpenSuitcaseResponse(OpenSuitcaseResponse.FAILED)

        return OpenSuitcaseResponse(OpenSuitcaseResponse.SUCCESS)

    def display_traj_(self, trajectory):
        print "Display trajectory"

        traj = DisplayTrajectory()
        traj.model_id = "shadow"
        traj.trajectory.joint_trajectory = trajectory
        traj.trajectory.joint_trajectory.header.frame_id = "world"
        traj.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
        self.display_traj_pub_.publish(traj)

        print "   -> trajectory published"
        time.sleep(0.5)


    def filter_traj_(self, motion_plan_res):
        try:
            req = FilterJointTrajectoryRequest()
            for name in ARM_NAMES:
                limit = JointLimits()
                limit.joint_name = name
                limit.min_position = -1.5
                limit.max_position = 1.5
                limit.has_velocity_limits = True
                limit.max_velocity = 0.1
                limit.has_acceleration_limits = True
                limit.max_acceleration = 0.1
                req.limits.append(limit)

            req.trajectory = motion_plan_res.trajectory.joint_trajectory
            req.allowed_time = rospy.Duration.from_sec( 5.0 )

            res = self.get_joint_state_.call()
            req.start_state.joint_state = res.joint_state

            res = self.trajectory_filter_.call( req )
        except rospy.ServiceException, e:
            rospy.logerr("Failed to filter "+str(e))
            return motion_plan_res.trajectory

        return res.trajectory

    def send_traj_(self, trajectory):
        print "Sending trajectory"

        traj = trajectory
        for index, point in enumerate(traj.points):
            #if index == 0 or index == len(traj.points) - 1:
            #    point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #else:
            #    point.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            point.time_from_start = rospy.Duration.from_sec(float(index) / 8.0)
        #    traj.points[index] = point
        self.send_traj_pub_.publish( traj )

        print "   -> trajectory sent"

if __name__ =="__main__":
    rospy.init_node("execution")
    execute = Execution()

    detected_suitcase = OpenSuitcase()

    rospy.spin()


