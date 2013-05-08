# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
PKG='rqt_sr_visual_servoing'
import roslib
roslib.load_manifest(PKG)

import os
import rospy, rospkg, actionlib
from sr_visual_servoing.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL
from python_qt_binding.QtGui import QWidget, QIcon, QStandardItemModel, QStandardItem

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
        # Get path to UI file which is in our packages resource directory
        rp = rospkg.RosPack()
        self.ui_file = os.path.join(rp.get_path(PKG), 'resource', 'gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(self.ui_file, self.ui)
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

        # Annoyingly setting the icon theme in designer only works in designer,
        # we have to set it again here for it to work at run time
        self.ui.startBtn.setIcon(QIcon.fromTheme('media-playback-start'))
        self.ui.stopBtn.setIcon(QIcon.fromTheme('media-playback-stop'))

        # Add widget to the user interface
        context.add_widget(self.ui)

        # Setup a model for the feedback and link to the view.
        self.feedback_model = QStandardItemModel(0,2)
        self.feedback_model.setHorizontalHeaderLabels(['Name','Value'])
        self.ui.feedbackView.setModel(self.feedback_model)
        self.ui.connect( self.ui, SIGNAL('feedback(QString)'), self.update_feedback )

        # ROS setup
        self.last_feedback = None
        self.client = actionlib.SimpleActionClient('visual_servo', VisualServoingAction)
        msg = ""
        if self.client.wait_for_server(rospy.Duration(2.0)):
            msg = "Found action server, servoing appears to be running"
            rospy.loginfo(msg)
        else:
            msg = "Can't find action server, servoing not running"
            rospy.logerr(msg)
        self.ui.statusValue.setText(msg)

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
        self.ui.statusValue.setText("Starting")

    def stop_clicked(self):
        self.client.cancel_all_goals()
        self.ui.statusValue.setText("Stopped")
        self.feedback_model.clear()

    def _feedback_cb(self, feedback):
        # We can't update the UI in this thread so stash the data and emit a
        # signal that can be traped by the main thread and update the ui.
        self.last_feedback = feedback
        self.ui.emit( SIGNAL('feedback(QString)'), "" )

    def update_feedback(self, data):
        """Listen for feedback signals and update the interface."""
        fb = self.last_feedback
        self.ui.statusValue.setText(str(self.client.get_goal_status_text()))

        # Update the feedback model, which triggers the view to update
        m = self.feedback_model
        m.setHorizontalHeaderLabels(['Name','Value'])

        m.setItem(0,0,QStandardItem('distance'))
        m.setItem(0,1,QStandardItem(str(fb.distance)))

        m.setItem(1,0,QStandardItem('object_pose.position.x'))
        m.setItem(1,1,QStandardItem(str(fb.object_pose.position.x)))
        m.setItem(2,0,QStandardItem('object_pose.position.y'))
        m.setItem(2,1,QStandardItem(str(fb.object_pose.position.y)))
        m.setItem(3,0,QStandardItem('object_pose.position.z'))
        m.setItem(3,1,QStandardItem(str(fb.object_pose.position.z)))

        m.setItem(4,0,QStandardItem('object_pose.orientation.x'))
        m.setItem(4,1,QStandardItem(str(fb.object_pose.orientation.x)))
        m.setItem(5,0,QStandardItem('object_pose.orientation.y'))
        m.setItem(5,1,QStandardItem(str(fb.object_pose.orientation.y)))
        m.setItem(6,0,QStandardItem('object_pose.orientation.z'))
        m.setItem(6,1,QStandardItem(str(fb.object_pose.orientation.z)))
        m.setItem(7,0,QStandardItem('object_pose.orientation.w'))
        m.setItem(7,1,QStandardItem(str(fb.object_pose.orientation.w)))

        m.setItem(8,0,QStandardItem('grasp_pose.position.x'))
        m.setItem(8,1,QStandardItem(str(fb.grasp_pose.position.x)))
        m.setItem(9,0,QStandardItem('grasp_pose.position.y'))
        m.setItem(9,1,QStandardItem(str(fb.grasp_pose.position.y)))
        m.setItem(10,0,QStandardItem('grasp_pose.position.z'))
        m.setItem(10,1,QStandardItem(str(fb.grasp_pose.position.z)))

        m.setItem(11,0,QStandardItem('grasp_pose.orientation.x'))
        m.setItem(11,1,QStandardItem(str(fb.grasp_pose.orientation.x)))
        m.setItem(12,0,QStandardItem('grasp_pose.orientation.y'))
        m.setItem(12,1,QStandardItem(str(fb.grasp_pose.orientation.y)))
        m.setItem(13,0,QStandardItem('grasp_pose.orientation.z'))
        m.setItem(13,1,QStandardItem(str(fb.grasp_pose.orientation.z)))
        m.setItem(14,0,QStandardItem('grasp_pose.orientation.w'))
        m.setItem(14,1,QStandardItem(str(fb.grasp_pose.orientation.w)))

        self.ui.feedbackView.resizeColumnsToContents()

