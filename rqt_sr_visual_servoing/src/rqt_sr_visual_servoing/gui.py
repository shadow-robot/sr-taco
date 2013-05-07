# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
roslib.load_manifest('rqt_sr_visual_servoing')

import os
import rospy, actionlib
from sr_visual_servoing.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL
from python_qt_binding.QtGui import QWidget

class RqtSrVisualServoing(Plugin):

    def __init__(self, context):
        super(RqtSrVisualServoing, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtSrVisualServoing')

        # # Process standalone plugin command-line arguments
        # from argparse import ArgumentParser
        # parser = ArgumentParser()
        # # Add argument(s) to the parser.
        # parser.add_argument("-q", "--quiet", action="store_true",
        #               dest="quiet",
        #               help="Put plugin in silent mode")
        # args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        # Create QWidget
        self.ui = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.ui)
        # Give QObjects reasonable names
        self.ui.setObjectName('RqtSrVisualServoingUi')
        # Show ui.windowTitle on left-top of each plugin (when it's set in ui).
        # This is useful when you open multiple plugins at once. Also if you
        # open multiple instances of your plugin at once, these lines add
        # number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            self.ui.setWindowTitle(self.ui.windowTitle() + (' (%d)' % context.serial_number()))

        # Wire up the buttons
        self.ui.startBtn.clicked.connect( self.start_clicked )
        self.ui.stopBtn.clicked.connect( self.stop_clicked )

        self.ui.connect( self.ui, SIGNAL('feedback(QString)'), self.feedback )

        # Add widget to the user interface
        context.add_widget(self.ui)

        # ROS setup
        self.last_feedback = None
        self.client = actionlib.SimpleActionClient('visual_servo', VisualServoingAction)
        if self.client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Found action server, servoing appears to be running")
        else:
            rospy.logerr("Can't find action server, servoing not running")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog

    def start_clicked(self):
        goal = VisualServoingActionGoal()
        self.client.send_goal(goal, feedback_cb = self._feedback_cb)

    def stop_clicked(self):
        self.client.cancel_all_goals()

    def _feedback_cb(self, feedback):
        # We can't update the UI in this thread so stash the data and emit a
        # signal that can be traped by the main thread and update the ui.
        self.last_feedback = feedback
        self.ui.emit( SIGNAL('feedback(QString)'), "" )

    def feedback(self, data):
        """Listen for feedback signals and update the interface."""
        fb = self.last_feedback
        self.ui.distanceValue.setText(str(fb.distance))

