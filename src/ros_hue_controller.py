#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ref:
#   https://github.com/studioimaginaire/phue
import sys
from os import path
import json

sys.path.append(path.abspath(path.dirname(__file__)) + "/../lib")

import rospy
from ros_hue.srv import HueTrigger, HueTriggerResponse
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
        rospy.Service(rospy.get_param(SERVICE_NAME_ON), HueTrigger, self.turn_on)
        rospy.Service(rospy.get_param(SERVICE_NAME_OFF), HueTrigger, self.turn_off)
        rospy.Service(rospy.get_param(SERVICE_NAME_TOGGLE), HueTrigger, self.toggle)
        return self

    def discover(self, req):
        self._bridge.connect()
        return TriggerResponse(success=True, message="")

    def turn_on(self, req):
        rospy.logdebug("Turn on")

        group_ids = [g.group_id for g in self._bridge.groups if g.name.count(req.group_name)]
        scene_ids = [s.scene_id for s in self._bridge.scenes if s.name.count(req.scene_name)]
        if req.is_scene:
            g_id = group_ids[0] if req.is_group and len(group_ids) > 0 else 0
            if len(scene_ids) < 1:
                return HueTriggerResponse(success=False,
                                          message="Not found the scene: " + req.scene_name)
            self._bridge.activate_scene(g_id, scene_ids[0])
            return HueTriggerResponse(success=True, message="")

        param = {"on": True}
        if req.is_hsv:
            param.update({"hue": max(req.hue, 65534)}) if req.hue >= 0 else param.update({"hue_inc": 0})
            param.update({"sat": max(req.sat, 254)}) if req.sat >= 0 else param.update({"sat_inc": 0})
            param.update({"bri": max(req.bri, 254)}) if req.bri >= 0 else param.update({"bri_inc": 0})

        if req.is_group:
            if len(group_ids) == 0:
                return HueTriggerResponse(success=False,
                                          message="Not found the group: " + req.group_name)
            self._bridge.set_group(group_ids, param, transitiontime=0)
        else:
            ids = self._light_ids if len(req.ids) == 0 else req.ids
            self._bridge.set_light(ids, param, transitiontime=0)

        return HueTriggerResponse(success=True, message="")

    def turn_off(self, req):
        param = {"on": False}
        if req.is_group:
            group_ids = [g.group_id for g in self._bridge.groups if g.name.count(req.group_name)]
            if len(group_ids) == 0:
                return HueTriggerResponse(success=False,
                                          message="Not found the group: " + req.group_name)
            self._bridge.set_group(group_ids, param, transitiontime=0)
        else:
            ids = self._light_ids if len(req.ids) == 0 else req.ids
            self._bridge.set_light(ids, param, transitiontime=0)

        return HueTriggerResponse(success=True, message="")

    def toggle(self, req):
        val = not self.is_on()
        return self.turn_on(req) if val else self.turn_off(req)

    def is_on(self):
        states = self._bridge.get_light()
        return all([states[str(id)]["state"]["on"] for id in self._light_ids])


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True, log_level=rospy.DEBUG)
    hue_controller = ROSHueController().activate()
    rospy.spin()
