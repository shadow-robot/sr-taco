#!/usr/bin/env python
#
# Copyright 2012 Shadow Robot Company Ltd.
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


import os
import roslib
roslib.load_manifest('sr_moving_object')
import rospy

from qt_gui.plugin import Plugin
from qt_gui.qt_binding_helper import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QShortcut, QMessageBox, QWidget
from gazebo_msgs.srv import ApplyBodyWrench, GetModelProperties, GetWorldProperties, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Wrench, Pose, Twist


'''
rosservice call /gazebo/apply_body_wrench '{body_name: "simple_box::my_box", wrench: {force: { x: 0.01, y: 0, z: 0 } }, start_time: 0, duration: 1000000000 }'
success: True
status_message: ''


rosservice call /gazebo/get_world_properties
sim_time: 47106.871
model_names: ['plane1_model', 'simple_box']
rendering_enabled: True
success: True
status_message: GetWorldProperties: got properties
toni@proteus:~$ rosservice call /gazebo/get_world_properties
sim_time: 51913.157
model_names: ['plane1_model', 'simple_box']
rendering_enabled: True
success: True
status_message: GetWorldProperties: got properties





rosservice call /gazebo/get_model_properties simple_box
parent_model_name: ''
canonical_body_name: ''
body_names: ['my_box']
geom_names: ['my_box_geom']
joint_names: []
child_model_names: []
is_static: False
success: True
status_message: GetModelProperties: got properties
'''

class SrMoveObject(Plugin):

    def __init__(self, context):
        super(SrMoveObject, self).__init__(context)
        self.setObjectName('SrMoveObject')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../uis/SrMoveObjects.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrMoveObjectsUi')
        context.add_widget(self._widget)

        #attaching the button press event to their actions
        self._widget.btnRefreshList.pressed.connect(self.on_refresh_list_clicked_)
        self._widget.btnZero.pressed.connect(self.on_zero_clicked_)
        self._widget.btnOpposite.pressed.connect(self.on_opposite_clicked_)
        self._widget.btn_wrench.pressed.connect(self.on_apply_wrench_clicked_)
        self._widget.btnSetToPos.pressed.connect(self.on_set_to_position_clicked_)

    def on_zero_clicked_(self):
        self._widget.lineEdit.setText("0.0")
        self._widget.lineEdit_2.setText("0.0")
        self._widget.lineEdit_3.setText("0.0")

    def on_opposite_clicked_(self):
        self._widget.lineEdit.setText( str((-1) * float(str(self._widget.lineEdit.text()))) )
        self._widget.lineEdit_2.setText( str((-1) * float(str(self._widget.lineEdit_2.text()))) )
        self._widget.lineEdit_3.setText( str((-1) * float(str(self._widget.lineEdit_3.text()))) )

    def on_set_to_position_clicked_(self):
        success = True
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = str(self._widget.comboBoxObjectList.currentText()).split('::')[0]
        model_state.pose = Pose()
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        
        model_state.pose.position.x = float(str(self._widget.lineEdit_4.text()))
        model_state.pose.position.y = float(str(self._widget.lineEdit_5.text()))
        model_state.pose.position.z = float(str(self._widget.lineEdit_6.text()))

        try:
            resp1 = set_model_state(model_state)
        except rospy.ServiceException:
            success = False
        if success:
            if not resp1.success:
                success = False

        if not success:
            QMessageBox.warning(self._widget, "Warning", "Could not set model state.")

    def on_refresh_list_clicked_(self):
        self._widget.comboBoxObjectList.clear()
        
        success = True
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        object_list = list()
        try:
            resp1 = get_world_properties()
        except rospy.ServiceException:
            success = False
        if success:
            get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            for model in resp1.model_names:
                try:
                    model_properties = get_model_properties(model)
                except rospy.ServiceException:
                    success = False
                if success:
                    for body in model_properties.body_names:
                        object_list.append(model + '::' + body)
        self._widget.comboBoxObjectList.addItems(object_list)
        if not success:
            QMessageBox.warning(self._widget, "Warning", "Could not load object list from Gazebo.")

    def on_apply_wrench_clicked_(self):
        success = True
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        body_name = self._widget.comboBoxObjectList.currentText()
        wrench = Wrench()
        wrench.force.x = float(str(self._widget.lineEdit.text()))
        wrench.force.y = float(str(self._widget.lineEdit_2.text()))
        wrench.force.z = float(str(self._widget.lineEdit_3.text()))

        try:
            resp1 = apply_body_wrench(body_name, "", None, wrench, rospy.Time.from_sec(0), rospy.Duration.from_sec(1.0))
        except rospy.ServiceException:
            success = False
        if success:
            if not resp1.success:
                success = False

        if not success:
            QMessageBox.warning(self._widget, "Warning", "Could not apply wrench to selected object.")



    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._unregisterPublisher()

    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass

