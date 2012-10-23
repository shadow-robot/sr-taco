import os
import roslib
roslib.load_manifest('sr_pcl_tracking_gui')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import *

from std_srvs.srv import *
from sr_pcl_tracking.srv import *

class SrPclTrackingProxy(object):
    """Provides a proxy interface to the sr_pcl_tracking/sr_pcl_tracker node.
    Call methods on an instance calls services etc on the tracker.
    Pass the node to proxy to by passing the name to the constructor. Default
    is '/sr_pcl_tracker'.
    """
    def __init__(self, topic = "/sr_pcl_tracker"):
        self.topic = topic

    def callEmptySrv(self, name):
        rospy.wait_for_service(name,10);
        try:
            srv_proxy = rospy.ServiceProxy(name, Empty)
            resp = srv_proxy()
            return resp
        except rospy.ServiceException, e:
            rospy.logerr("%s service fail: %s"%(name, e))

    def callSrv(self, name, msgtype, *args):
        rospy.wait_for_service(name);
        try:
            srv_proxy = rospy.ServiceProxy(name, msgtype)
            resp = srv_proxy(*args)
            return resp
        except rospy.ServiceException, e:
            rospy.logerr("%s service fail: %s"%(name, e))

    def track_centered(self):
        self.callEmptySrv(self.topic + "/track_centered")

    def track_nearest(self):
        self.callEmptySrv(self.topic + "/track_nearest")

    def list_references(self):
        req  = ListReferenceRequest()
        return self.callSrv(self.topic+"/list_reference", ListReference, req)
    
    def load_reference(self, name):
        return self.callSrv(self.topic+"/load_reference", LoadReference, name)

    def save_reference(self, name):
        return self.callSrv(self.topic+"/save_reference", SaveReference, name)


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
        self.ui.refreshBtn.pressed.connect(self.refresh)
        self.ui.loadBtn.pressed.connect(self.load_selected)
        self.ui.saveBtn.pressed.connect(self.save_selected)
        self.ui.saveAsBtn.pressed.connect(self.save_as)


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

    def show_msg(self,msg):
        """Show the user a message box."""
        msg_box = QMessageBox(self.ui)
        msg_box.setText("No reference selected. Please select one from the list.")
        msg_box.show()

    def refresh(self):
        """Refresh the list of references."""
        self.ui.referenceList.clear()
        resp = self.tracker.list_references()
        for name in resp.names:
            self.ui.referenceList.addItem(name)

    def load_selected(self):
        """Load the currently selected reference."""
        item = self.ui.referenceList.currentItem();
        if not item:
            return self.show_msg("No reference selected. Please select one from the list.")
        self.tracker.load_reference(item.text())

    def save_selected(self):
        """Save the current reference using the currently selected name."""
        item = self.ui.referenceList.currentItem();
        if not item:
            return self.show_msg("No reference selected. Please select one from the list.")
        self.tracker.save_reference(item.text())

    def save_as(self):
        """Save the current reference using a new name, requested in a dialog."""
        text, ok = QInputDialog.getText(self.ui, 'Save Reference As', 'Enter name for object:')
        if ok:
            self.tracker.save_reference(text)
            self.refresh()


