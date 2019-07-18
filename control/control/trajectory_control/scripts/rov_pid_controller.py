#!/usr/bin/env python
#***************************************************
# * Title: UUV Simulator
# * Author: The UUV Simulator Authors
# * Date: 2016
# * Availability: https://uuvsimulator.github.io/
#***************************************************

import rospy
import numpy as np
from control_interfaces import DPPIDControllerBase


class ROV_PIDController(DPPIDControllerBase):
    """PID controller for the dynamic positioning of ROVs."""

    _LABEL = 'PID'
    def __init__(self):
        self._tau = np.zeros(6)
        DPPIDControllerBase.__init__(self, False)
        self._is_init = True

    def update_controller(self):
        if not self._is_init:
            return False
        # Update PID control action
        self._tau = self.update_pid()
        self.publish_control_wrench(self._tau)
        return True

if __name__ == '__main__':
    print('Starting PID')
    rospy.init_node('rov_pid_controller')

    try:
        node = ROV_PIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
