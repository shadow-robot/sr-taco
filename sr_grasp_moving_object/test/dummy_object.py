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

import roslib
roslib.load_manifest('sr_grasp_moving_object')
import rospy
import random

from geometry_msgs.msg import PoseStamped

class DummyMovingObject(object):
    """
    This simply publishes a dummy moving object trajectory.
    Also displays it in rviz.
    """

    def __init__(self, ):
        """
        """
        self.publisher = rospy.Publisher("~position", PoseStamped)
        self.msg = PoseStamped()
        self.msg.header.frame_id="/shadowarm_base"
        self.msg.pose.orientation.w = 1.0
        self.msg.pose.position.z = 0.1
        self.msg.pose.position.x = 0.5
        self.msg.pose.position.y = 0.0

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        self.timestep = 0.0

    def activate(self, rate = 10):
        self.timestep = 1.0 / rate
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.publisher.publish(self.msg)

        self.msg.header.stamp = rospy.Time.now()

        #we randomize the velocity a bit
        self.velocity_x += (random.random() - 0.5) / 150.0
        self.velocity_y += (random.random() - 0.5) / 150.0
        self.velocity_z += (random.random() - 0.5) / 150.0

        self.msg.pose.position.x = self.msg.pose.position.x + self.velocity_x * self.timestep
        #reverse the speed if we're out of a reasonable box
        if self.msg.pose.position.x > 0.7 or self.msg.pose.position.x < 0.3:
            self.velocity_x = - self.velocity_x

        self.msg.pose.position.y = self.msg.pose.position.y + self.velocity_y * self.timestep
        #reverse the speed if we're out of a reasonable box
        if self.msg.pose.position.y > 0.4 or self.msg.pose.position.y < -0.4:
            self.velocity_y = - self.velocity_y

        self.msg.pose.position.z = self.msg.pose.position.z + self.velocity_z * self.timestep
        #reverse the speed if we're out of a reasonable box
        if self.msg.pose.position.z > 0.3 or self.msg.pose.position.z < -0.1:
            self.velocity_z = - self.velocity_z


if __name__ == "__main__":
    rospy.init_node("object")
    dmo = DummyMovingObject()
    dmo.activate( 10 )
