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
        self.msg.pose.orientation.w = 1.0
        self.msg.pose.position.z = 1.0
        self.msg.pose.position.x = 1.0
        self.msg.pose.position.y = 1.0
        self.going_back = True

    def activate(self, rate = 10):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.publisher.publish(self.msg)

        self.msg.header.stamp = rospy.Time.now()
        if self.going_back:
            self.msg.pose.position.x -= 0.02
            self.msg.pose.position.y -= 0.02
            if self.msg.pose.position.x < 0.0:
                self.going_back = False
        else:
            self.msg.pose.position.x += 0.02
            self.msg.pose.position.y += 0.02
            if self.msg.pose.position.x > 1.0:
                self.going_back = True

if __name__ == "__main__":
    rospy.init_node("object")
    dmo = DummyMovingObject()
    dmo.activate()
