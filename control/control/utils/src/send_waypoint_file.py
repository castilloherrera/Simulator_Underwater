#!/usr/bin/env python
#***************************************************
# * Title: UUV Simulator
# * Author: The UUV Simulator Authors
# * Date: 2016
# * Availability: https://uuvsimulator.github.io/
#***************************************************

import rospy
import sys
from control_msgs.srv import InitWaypointsFromFile
from std_msgs.msg import String, Time


if __name__ == '__main__':
    rospy.init_node('send_waypoint_file')
    rospy.loginfo('Send a waypoint file, namespace=%s', rospy.get_namespace())

    if rospy.is_shutdown():
        rospy.logerr('ROS master not running!')
        sys.exit(-1)

    if rospy.has_param('~filename'):
        filename = rospy.get_param('~filename')
    else:
        raise rospy.ROSException('No filename found')

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = True
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            rospy.logerr('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
        else:
            start_now = False
    else:
        start_now = True

    rospy.loginfo('Start time=%.2f s' % start_time)

    interpolator = rospy.get_param('~interpolator', 'lipb')

    try:
        rospy.wait_for_service('init_waypoints_from_file', timeout=30)
    except rospy.ROSException:
        raise rospy.ROSException('Service not available! Closing node...')

    try:
        init_wp = rospy.ServiceProxy(
            'init_waypoints_from_file',
            InitWaypointsFromFile)
    except rospy.ServiceException as e:
        raise rospy.ROSException('Service call failed, error=%s', str(e))

    success = init_wp(Time(rospy.Time.from_sec(start_time)),
                      start_now,
                      String(filename),
                      String(interpolator))

    if success:
        rospy.loginfo('Waypoints file successfully received, '
                      'filename=%s', filename)
    else:
        rospy.loginfo('Failed to send waypoints')
