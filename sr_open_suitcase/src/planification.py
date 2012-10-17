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

from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanRequest, GetMotionPlanResponse, SetPlanningSceneDiff, FilterJointTrajectory, FilterJointTrajectoryRequest
from arm_navigation_msgs.msg import MotionPlanRequest, Shape, PositionConstraint, OrientationConstraint, DisplayTrajectory, Constraints, JointConstraint, JointLimits, RobotState
from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams
#import interpolated_ik_motion_planner.ik_utilities as ik_utilities
from kinematics_msgs.srv import GetConstraintAwarePositionIK
from kinematics_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sr_utilities.srv import getJointState

import time
import math

ARM_NAMES = ['ShoulderJRotate', 'ShoulderJSwing', 'ElbowJSwing', 'ElbowJRotate', "WRJ1", "WRJ2"]
ARM_IK_SEED = [-0.0011556513918362654, 0.33071140061761284, 1.9152039367468952, 0.008440334101951663, 0.0, 0.0]

class Planification(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.loginfo("Waiting for services /ompl_planning/plan_kinematic_path, /environment_server/set_planning_scene_diff, /shadow_right_arm_kinematics/get_constraint_aware_ik ...")

        rospy.wait_for_service("/ompl_planning/plan_kinematic_path")
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        rospy.wait_for_service("/shadow_right_arm_kinematics/get_constraint_aware_ik")
        rospy.wait_for_service("/r_interpolated_ik_motion_plan_set_params")
        rospy.wait_for_service("/r_interpolated_ik_motion_plan")
        rospy.wait_for_service("/trajectory_filter_unnormalizer/filter_trajectory")
        rospy.wait_for_service("/getJointState")
        rospy.loginfo("  OK services found")

        self.set_planning_scene_diff_ = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        self.planifier_ = rospy.ServiceProxy('/ompl_planning/plan_kinematic_path', GetMotionPlan)
        self.constraint_aware_ik_ = rospy.ServiceProxy("/shadow_right_arm_kinematics/get_constraint_aware_ik", GetConstraintAwarePositionIK)
        self.interpolated_ik_params_srv = rospy.ServiceProxy('/r_interpolated_ik_motion_plan_set_params', SetInterpolatedIKMotionPlanParams)
        self.interpolated_ik_srv = rospy.ServiceProxy('/r_interpolated_ik_motion_plan', GetMotionPlan)
        #self.ik_utils = ik_utilities.IKUtilities('right',None,0) # do not wait for service this is not needed for us
        self.get_joint_state_ = rospy.ServiceProxy("/getJointState", getJointState)

        #self.standard_ik_ = rospy.ServiceProxy("/shadow_right_arm_kinematics/get_ik", GetPositionIK)


    def plan_arm_motion(self, arm_name, planner, palm_target_pose, hand_target_posture=[]):
        """Plan motion for the palm, eventually virtually setting the hand in the pre-grasp posture to manage grasping in small spaces
        """
        goal = PoseStamped()
        goal.header.frame_id = "world"

        goal.pose=palm_target_pose.pose

        #goal.pose.position.x = 0.470
        #goal.pose.position.y = -0.166
        #goal.pose.position.z = 1.665

        #goal.pose.orientation.x = 0.375
        #goal.pose.orientation.y = 0.155
        #goal.pose.orientation.z = 0.844
        #goal.pose.orientation.w = 0.351

        if(planner=="cartesianspace"):
            result=self.plan_motion_cartesian_( arm_name, goal )
        else:
            result=self.plan_motion_joint_state_( arm_name, goal )

        return result

    def plan_motion_joint_state_(self, arm_name, goal_pose, link_name = "palm"):
        self.reset_planning_scene_()

        motion_plan_res= GetMotionPlanResponse()

        #first get the ik for the pose we want to go to
        ik_solution = None
        try:
            req = PositionIKRequest()
            req.ik_link_name = link_name
            req.pose_stamped = goal_pose
            req.ik_seed_state.joint_state.name = ARM_NAMES
            req.ik_seed_state.joint_state.position = ARM_IK_SEED
            ik_solution = self.constraint_aware_ik_.call( req, Constraints(), rospy.Duration(5.0) )
        except rospy.ServiceException, e:
            rospy.logerr( "Failed to compute IK "+str(e) )
            motion_plan_res.error_code.val = motion_plan_res.error_code.NO_IK_SOLUTION
            return motion_plan_res

        if ik_solution.error_code.val != ik_solution.error_code.SUCCESS:
            rospy.logerr("couldn't find an ik solution to go to: " + str(goal_pose))
            motion_plan_res.error_code.val = ik_solution.error_code.val
            return motion_plan_res

        #motion_plan_res = None
        #try to plan the motion to the target joint_state
        try:
            motion_plan_request = MotionPlanRequest()

            motion_plan_request.group_name = arm_name #"right_arm"
            motion_plan_request.num_planning_attempts = 1
            motion_plan_request.planner_id = ""
            motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

            motion_plan_request.expected_path_duration = rospy.Duration(5.0)
            motion_plan_request.expected_path_dt = rospy.Duration(0.5)

            # set joint_constraints
            for target, name in zip(ik_solution.solution.joint_state.position, ik_solution.solution.joint_state.name):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = target
                joint_constraint.tolerance_below = 0.1
                joint_constraint.tolerance_above = 0.1

                motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)

            # start the planner
            motion_plan_res = self.planifier_( motion_plan_request )

            if motion_plan_res.error_code.val != motion_plan_res.error_code.SUCCESS:
                rospy.logerr("The planning failed: " + str(motion_plan_res.error_code.val))
            else:
                # compute velocity and appropriate times
                (times, vels) = self.trajectory_times_and_vels(motion_plan_res.trajectory, [.1]*6, [.2]*6)
                #print times
                #print vels
                for i in range(len(motion_plan_res.trajectory.joint_trajectory.points)):
                    motion_plan_res.trajectory.joint_trajectory.points[i].velocities = vels[i]
                    motion_plan_res.trajectory.joint_trajectory.points[i].time_from_start = rospy.Duration(times[i])
                #print motion_plan_res.trajectory.joint_trajectory
        except rospy.ServiceException, e:
            rospy.logerr( "Failed to plan "+str(e) )
            motion_plan_res.error_code.val = motion_plan_res.error_code.PLANNING_FAILED

        return motion_plan_res

    def plan_motion_cartesian_(self, arm_name, goal_pose, link_name="palm"):
        self.reset_planning_scene_()

        motion_plan_res = None
        try:
            motion_plan_request = MotionPlanRequest()

            motion_plan_request.group_name = "right_arm_cartesian"
            motion_plan_request.num_planning_attempts = 1
            motion_plan_request.planner_id = ""
            motion_plan_request.allowed_planning_time = rospy.Duration(120.0)

            position_constraint = PositionConstraint()
            position_constraint.header.stamp = rospy.Time.now()
            position_constraint.header.frame_id = goal_pose.header.frame_id
            position_constraint.link_name = link_name
            position_constraint.position = goal_pose.pose.position
            position_constraint.constraint_region_shape.type = Shape.BOX
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.weight = 1.0
            #position_constraint.constraint_region_orientation.x = 0.307
            #position_constraint.constraint_region_orientation.y = 0.127
            #position_constraint.constraint_region_orientation.z = 0.871
            #position_constraint.constraint_region_orientation.w = 0.362
            position_constraint.constraint_region_orientation.w = 1.0
            motion_plan_request.goal_constraints.position_constraints.append(position_constraint)

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.stamp = rospy.Time.now()
            orientation_constraint.header.frame_id = goal_pose.header.frame_id
            orientation_constraint.link_name = link_name

            orientation_constraint.orientation.x = 0.351
            orientation_constraint.orientation.y = 0.155
            orientation_constraint.orientation.z = 0.844
            orientation_constraint.orientation.w = 0.375

            #orientation_constraint.orientation.x = 0.454
            #orientation_constraint.orientation.y = 0.665
            #orientation_constraint.orientation.z = 0.499
            #orientation_constraint.orientation.w = 0.320

            orientation_constraint.absolute_roll_tolerance = 1.5
            orientation_constraint.absolute_pitch_tolerance = 1.5
            orientation_constraint.absolute_yaw_tolerance = 1.5
            orientation_constraint.weight = 0.5
            motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

            motion_plan_res = self.planifier_( motion_plan_request )

            if motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS:
                self.display_traj_( motion_plan_res )
            else:
                rospy.logerr("The planning failed: " + str(motion_plan_res.error_code.val))

        except rospy.ServiceException, e:
            rospy.logerr( "Failed to plan "+str(e) )
            return False

    def get_interpolated_ik_motion_plan(self, start_pose, target_pose, collision_check=False,
                                        steps_before_abort=1, num_steps=0,
                                        frame='shadowarm_base', max_joint_vels=[0.1]*6, max_joint_accs=[0.5]*6):
        self.reset_planning_scene_()

        ik_motion_plan_res = self.interpolated_ik_params_srv(num_steps,
                                              math.pi/7.0,
                                              1,
                                              steps_before_abort,
                                              0.02,
                                              0.1,
                                              collision_check,
                                              1, #start from end
                                              max_joint_vels,
                                              max_joint_accs)


        ik_motion_plan_req = GetMotionPlanRequest()
        ik_motion_plan_req.motion_plan_request.start_state.joint_state.name = ARM_NAMES

        joint_state_res = self.get_joint_state_.call()
        #start_angles = res.joint_state.positions # one cannot use this directly it contains not only arm but also fingers
        ik_motion_plan_req.motion_plan_request.start_state.joint_state.position = [0.3]*6 #start_angles
        ik_motion_plan_req.motion_plan_request.start_state.multi_dof_joint_state.poses = [start_pose.pose]
        ik_motion_plan_req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_ids = ["palm"]
        ik_motion_plan_req.motion_plan_request.start_state.multi_dof_joint_state.frame_ids = [start_pose.header.frame_id]

        pos_constraint = PositionConstraint()
        pos_constraint.position = target_pose.pose.position
        pos_constraint.header.frame_id = target_pose.header.frame_id
        ik_motion_plan_req.motion_plan_request.goal_constraints.position_constraints = [pos_constraint]

        orient_constraint = OrientationConstraint()
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.header.frame_id = target_pose.header.frame_id
        ik_motion_plan_req.motion_plan_request.goal_constraints.orientation_constraints = [orient_constraint]

        #if ordered_collision_operations is not None:
        #    ik_motion_plan_req.motion_plan_request.ordered_collision_operations = ordered_collision_operations

        ik_motion_plan_res = self.interpolated_ik_srv(ik_motion_plan_req)
        return ik_motion_plan_res

    def trajectory_times_and_vels(self, traj, max_joint_vels = [.2]*6, max_joint_accs = [.5]*6):

        #min time for each segment
        min_segment_time = .01

        if not traj:
            rospy.logdebug("traj path was empty!")
            return([], [])
        traj_length = len(traj.joint_trajectory.points)
        num_joints = len(traj.joint_trajectory.joint_names)

        #sanity-check max vels and accelerations
        if not max_joint_vels:
            max_joint_vels = [.2]*7
        elif len(max_joint_vels) != num_joints:
            rospy.logerr("invalid max_joint_vels!")
            return ([], [])
        if not max_joint_accs:
            max_joint_accs = [.5]*7
        elif len(max_joint_accs) != num_joints:
            rospy.logerr("invalid max_joint_accs!")
            return ([], [])
        for ind in range(num_joints):
            if max_joint_vels[ind] <= 0.:
                max_joint_vels[ind] = .2
            if max_joint_accs[ind] <= 0.:
                max_joint_accs[ind] = .5

        vels = [[None]*num_joints for i in range(traj_length)]

        #give the trajectory a bit of time to start
        segment_times = [None]*traj_length
        segment_times[0] = 0.05

        #find vaguely appropriate segment times, assuming that we're traveling at max_joint_vels at the fastest joint
        for ind in range(traj_length-1):
            joint_diffs = [math.fabs(traj.joint_trajectory.points[ind+1].positions[x]-traj.joint_trajectory.points[ind].positions[x]) for x in range(num_joints)]
            joint_times = [diff/vel for (diff, vel) in zip(joint_diffs, max_joint_vels)]
            segment_times[ind+1] = max(joint_times+[min_segment_time])

        #set the initial and final velocities to 0 for all joints
        vels[0] = [0.]*num_joints
        vels[traj_length-1] = [0.]*num_joints

        #also set the velocity where any joint changes direction to be 0 for that joint
        #and otherwise use the average velocity (assuming piecewise-linear velocities for the segments before and after)
        for ind in range(1, traj_length-1):
            for joint in range(num_joints):
                diff0 = traj.joint_trajectory.points[ind].positions[joint]-traj.joint_trajectory.points[ind-1].positions[joint]
                diff1 = traj.joint_trajectory.points[ind+1].positions[joint]-traj.joint_trajectory.points[ind].positions[joint]
                if (diff0>0 and diff1<0) or (diff0<0 and diff1>0):
                    vels[ind][joint] = 0.
                else:
                    vel0 = diff0/segment_times[ind]
                    vel1 = diff1/segment_times[ind+1]
                    vels[ind][joint] = (vel0+vel1)/2.

        #increase the times if the desired velocities would require overly large accelerations
        for ind in range(1, traj_length):
            for joint in range(num_joints):
                veldiff = math.fabs(vels[ind][joint]-vels[ind-1][joint])
                acc = veldiff/segment_times[ind]
                try:
                    if acc > max_joint_accs[joint]:
                        segment_times[ind] = veldiff/max_joint_accs[joint]
                except:
                    pdb.set_trace()

        #turn the segment_times into waypoint times (cumulative)
        times = [None]*traj_length
        times[0] = segment_times[0]
        for ind in range(1, traj_length):
            try:
                times[ind] = times[ind-1]+segment_times[ind]
            except:
                rospy.logerr("could not add segment time")
                continue

        #return the times and velocities
        return (times, vels)

    def reset_planning_scene_(self):
        try:
            self.set_planning_scene_diff_.call()
        except:
            rospy.logerr("failed to set the planning scene")

if __name__ =="__main__":
    rospy.init_node("planification")
    plan = Planification()

    object_pose = PoseStamped()
    object_pose.pose.position.x = 0.470
    object_pose.pose.position.y = 0.166
    object_pose.pose.position.z = 1.665

    object_pose.pose.orientation.x = 0.375
    object_pose.pose.orientation.y = 0.155
    object_pose.pose.orientation.z = 0.844
    object_pose.pose.orientation.w = 0.351


    result = plan.plan_arm_motion( "right_arm", "jointspace", object_pose )
    rospy.logerr("got result " + str(result))
    rospy.spin()

