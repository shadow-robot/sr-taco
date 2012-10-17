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
from sr_open_suitcase.srv import OpenSuitcase
from sr_open_suitcase.msg import Suitcase

if __name__ == "__main__":
    rospy.init_node("send_dummy_suitcase")
    suitcase = Suitcase()

    suitcase.header.frame_id = "/world"

    #where's the opening mechanism
    suitcase.opening_mechanism.pose_stamped.header.frame_id = "/world"
    suitcase.opening_mechanism.pose_stamped.pose.position.x = 0.39
    suitcase.opening_mechanism.pose_stamped.pose.position.y = 0.0
    suitcase.opening_mechanism.pose_stamped.pose.position.z = 1.08

    suitcase.opening_mechanism.pose_stamped.pose.orientation.x = 0.45
    suitcase.opening_mechanism.pose_stamped.pose.orientation.y = 0.55
    suitcase.opening_mechanism.pose_stamped.pose.orientation.z = 0.55
    suitcase.opening_mechanism.pose_stamped.pose.orientation.w = 0.45

    # suitcase.opening_mechanism.pose_stamped.pose.orientation.x = 0.375
    # suitcase.opening_mechanism.pose_stamped.pose.orientation.y = 0.155
    # suitcase.opening_mechanism.pose_stamped.pose.orientation.z = 0.844
    # suitcase.opening_mechanism.pose_stamped.pose.orientation.w = 0.351

    suitcase.opening_mechanism.dimensions.x = 0.05
    suitcase.opening_mechanism.dimensions.y = 0.05
    suitcase.opening_mechanism.dimensions.z = 0.05

    #where are the axes
    suitcase.lid_axis_a.x = 0.8
    suitcase.lid_axis_a.y = -0.3
    suitcase.lid_axis_a.z = 1.2

    suitcase.lid_axis_b.x = 0.8
    suitcase.lid_axis_b.y = 0.3
    suitcase.lid_axis_b.z = 1.2

    rospy.wait_for_service("/execution/open_suitcase")
    open_suitcase = rospy.ServiceProxy("/execution/open_suitcase", OpenSuitcase)
    try:
        resp = open_suitcase(suitcase)
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
