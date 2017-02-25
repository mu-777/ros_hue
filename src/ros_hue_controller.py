#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ref:
#   https://github.com/studioimaginaire/phue
import sys
import rospy
from ros_hue.srv import IDsTrigger, IDsTriggerResponse
from std_srvs.srv import Trigger, TriggerResponse

try:
    import phue
except ImportError:
    rospy.logerr('Not found pyhton package "phue"')
    rospy.logerr('-> You should do "pip install phue"')
    sys.exit()

try:
    import logging
    from my_common.rospy_logging import ConnectPythonLoggingToROS
except ImportError:
    rospy.logwarn("Fail to connect phue logger to ros")
else:
    phue.logger.addHandler(ConnectPythonLoggingToROS())
    phue.logger.setLevel(logging.DEBUG)

DEFAULT_NODE_NAME = 'hue_controller'

PARAM_NAME_IP = "/hue_ip"
PARAM_NAME_ID = "/hue_id"
PARAM_NAME_USERNAME = "/hue_username"
PARAM_NAME_LIGHT_IDS = "/hue_light_ids"

SERVICE_NAME_ON = "~on_srv"
SERVICE_NAME_OFF = "~off_srv"
SERVICE_NAME_TOGGLE = "~toggle_srv"
SERVICE_NAME_INITIALIZE = "~init_srv"


class ROSHueController(object):
    def __init__(self):
        self.is_activated = False
        self._bridge = phue.Bridge(rospy.get_param(PARAM_NAME_IP),
                                   rospy.get_param(PARAM_NAME_USERNAME))
        self._light_ids = rospy.get_param(PARAM_NAME_LIGHT_IDS)

    def activate(self):
        self.is_activated = True
        rospy.Service(rospy.get_param(SERVICE_NAME_INITIALIZE), Trigger, self.discover)
        rospy.Service(rospy.get_param(SERVICE_NAME_ON), IDsTrigger, self.switch(True))
        rospy.Service(rospy.get_param(SERVICE_NAME_OFF), IDsTrigger, self.switch(False))
        rospy.Service(rospy.get_param(SERVICE_NAME_TOGGLE), IDsTrigger, self.switch(None))
        return self

    def discover(self, req):
        self._bridge.connect()
        return TriggerResponse(success=True, message="")

    def switch(self, val):
        def _switch(req):
            req.ids = self._light_ids if len(req.ids) == 0 else req.ids
            rospy.loginfo("Turn " + ("on" if val else "off"))
            self._bridge.set_light(req.ids, "on", value=val, transitiontime=0)
            return IDsTriggerResponse(success=True, message="")

        def _toggle(req):
            val = not self.is_on()
            req.ids = self._light_ids if len(req.ids) == 0 else req.ids
            rospy.loginfo("Turn " + ("on" if val else "off"))
            self._bridge.set_light(req.ids, "on", value=val, transitiontime=0)
            return IDsTriggerResponse(success=True, message="")

        return _switch if val is not None else _toggle

    def is_on(self):
        return True


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    hue_controller = ROSHueController().activate()
    rospy.spin()
