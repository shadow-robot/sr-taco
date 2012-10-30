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
from planification import Planification
from sr_utilities.srv import getJointState
from visualization_msgs.msg import Marker
from tf import transformations
import actionlib

import math
import numpy as np

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
        self.markers_pub_ = rospy.Publisher("~markers", Marker)

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

        # access arm_movement actionlib
        self.joint_spline_trajectory_actionclient_ = actionlib.SimpleActionClient('/r_arm_controller/joint_trajectory_action', FollowJointTrajectoryAction)
        self.joint_spline_trajectory_actionclient_.wait_for_server()
        rospy.loginfo("joint_spline_trajectory server ready")

        self.suitcase_src_ = rospy.Service("~open_suitcase", OpenSuitcase, self.open_lid)

    def open_lid(self, suitcase_req):
        suitcase = suitcase_req.suitcase
        self.display_suitcase_(suitcase)

        semi_circle = self.go_to_mechanism_and_grasp_(suitcase)

        time.sleep(1.0)

        self.lift_lid_(suitcase, semi_circle)

    def go_to_mechanism_and_grasp_(self, suitcase):
        #compute the full trajectory
        semi_circle = self.compute_semi_circle_traj_(suitcase)

        #go to the first step (ie to the mechanism)
        self.plan_and_execute_step_(semi_circle[0])

        #then close the hand
        print "TODO: grasp the mechanism"

        #return the full traj
        return semi_circle


    def lift_lid_(self, suitcase, semi_circle):
        is_first = True
        for step in semi_circle:
            #ignore the first step as we're already there
            if is_first:
                is_first = False
                continue

            self.plan_and_execute_step_(step)
            time.sleep(0.5)

        return OpenSuitcaseResponse(OpenSuitcaseResponse.SUCCESS)

    def plan_and_execute_step_(self, step):
        motion_plan_res = self.plan.plan_arm_motion( "right_arm", "jointspace", step )
        if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
            rospy.logdebug("OK, motion planned, executing it.")
            # filter the trajectory
            filtered_traj = self.filter_traj_(motion_plan_res)
            #go there
            self.display_traj_( filtered_traj )
            self.send_traj_( filtered_traj )

        else:
            rospy.logerr("This step was impossible")

    def compute_semi_circle_traj_(self, suitcase, nb_steps = 10):
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
            # not rotating more than 40 as WRJ1 can't go further than this.
            rotation_angle = min( float(i) * math.pi / 2.0 / float(nb_steps), math.radians(20.0))

            ####
            # POSITION
            target.pose.position.x = axis[0] - ((axis[0] - mechanism[0]) * math.cos(rotation_angle))
            target.pose.position.z = axis[2] + ((axis[0] - mechanism[0]) * math.sin(rotation_angle))

            ####
            # ORIENTATION
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

    def recompute_timings_(self, motion_plan):
        start_time = rospy.Duration(0)
        last_start_time = rospy.Duration(0)
        for index, point in enumerate(motion_plan.trajectory.joint_trajectory.points):
            if point.time_from_start < last_start_time:
                start_time = last_start_time
            motion_plan.trajectory.joint_trajectory.points[index].time_from_start = start_time + point.time_from_start + rospy.Duration(5.0)
            last_start_time = point.time_from_start

        return motion_plan

    def display_traj_(self, trajectory):
        rospy.logdebug( "Display trajectory" )

        traj = DisplayTrajectory()
        traj.model_id = "shadow"
        traj.trajectory.joint_trajectory = trajectory
        traj.trajectory.joint_trajectory.header.frame_id = "world"
        traj.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
        self.display_traj_pub_.publish(traj)

        time.sleep(0.5)

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
        rospy.logdebug( "Sending trajectory" )
        #for index, point in enumerate(traj.points):
        #    if index == 0 or index == len(traj.points) - 1:
        #        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #    else:
        #        point.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        #    point.time_from_start = rospy.Duration.from_sec(float(index) / 8.0)
        #    traj.points[index] = point

        #prepare goal
        trajgoal = FollowJointTrajectoryGoal()
        trajgoal.trajectory = trajectory
        # send goal
        self.joint_spline_trajectory_actionclient_.send_goal(trajgoal)
        # wait for result up to 30 seconds
        self.joint_spline_trajectory_actionclient_.wait_for_result(timeout=rospy.Duration.from_sec(50))
        # analyze result
        joint_spline_trajectory_result_ = self.joint_spline_trajectory_actionclient_.get_result()
        if self.joint_spline_trajectory_actionclient_.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The joint_trajectory action has failed: " + str(joint_spline_trajectory_result_.error_code) )
            return -1
        else:
            rospy.logdebug("The joint_trajectory action has succeeded")
            return 0

if __name__ =="__main__":
    rospy.init_node("execution")
    execute = Execution()

    detected_suitcase = OpenSuitcase()

    rospy.spin()


