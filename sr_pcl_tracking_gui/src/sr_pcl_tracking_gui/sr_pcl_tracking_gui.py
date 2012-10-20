import os
import roslib
roslib.load_manifest('sr_pcl_tracking_gui')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from std_srvs.srv import *

class SrPclTrackingProxy(object):
    """Provides a proxy interface to the sr_pcl_tracking/sr_pcl_tracker node.
    Call methods on an instance calls services etc on the tracker.
    Pass the node to proxy to py passing the name to the constructor. Default
    is /sr_pcl_tracker
    """
    def __init__(self, topic = "/sr_pcl_tracker"):
        self.topic = topic

    def track_centered(self):
        self.callEmptySrv(self.topic + "/track_centered")

    def track_nearest(self):
        self.callEmptySrv(self.topic + "/track_nearest")

    def callEmptySrv(self, name):
        rospy.wait_for_service(name);
        try:
            srv_proxy = rospy.ServiceProxy(name, Empty)
            resp = srv_proxy()
            return resp
        except rospy.ServiceException, e:
            rospy.logerr("%s service fail: %s"%(name, e))


class SrPclTrackingGui(Plugin):

    def __init__(self, context):
        super(SrPclTrackingGui, self).__init__(context)
        # give QObjects reasonable names
        self.setObjectName('SrPclTrackingGui')

        self.tracker = SrPclTrackingProxy("/sr_pcl_tracker");

        # create QWidget
        self.ui = QWidget()
        # get path to UI file which is a sibling of this file
        # in this example the .ui file is in the same folder as this Python file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sr_pcl_tracking_gui.ui')
        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.ui)
        # give QObjects reasonable names
        self.ui.setObjectName('SrPclTrackingGuiUi')
        # add widget to the user interface
        context.add_widget(self.ui)

        self.ui.trackCenteredBtn.pressed.connect(self.tracker.track_centered)
        self.ui.trackNearestBtn.pressed.connect(self.tracker.track_nearest)

        #self.ui.refreshBtn
        #self.ui.saveBtn
        #self.ui.loadBtn

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
        # Usually used to open a dialog to offer the user a set of configuration
