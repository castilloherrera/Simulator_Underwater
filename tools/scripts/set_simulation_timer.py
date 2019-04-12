#***************************************************    
# * Title: UUV Simulator   
# * Author: The UUV Simulator Authors  
# * Date: 2016      
# * Availability: https://uuvsimulator.github.io/
#***************************************************

import rospy


if __name__ == '__main__':
    rospy.init_node('set_simulation_timer')

    if rospy.is_shutdown():
        rospy.ROSException('ROS master is not running!')

    timeout = 0.0
    if rospy.has_param('~timeout'):
        timeout = rospy.get_param('~timeout')
        if timeout <= 0:
            raise rospy.ROSException('Termination time must be a positive floating point value')

    print 'Starting simulation timer - Timeout = %2.f s' % timeout
    rate = rospy.Rate(100)
    while rospy.get_time() < timeout:
        rate.sleep()

    print 'Simulation timeout - Killing simulation...'
