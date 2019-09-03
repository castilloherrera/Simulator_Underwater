#***************************************************
# * Title: UUV Simulator
# * Author: The UUV Simulator Authors
# * Date: 2016
# * Availability: https://uuvsimulator.github.io/
#***************************************************

import rospy
import numpy as np
from tf.transformations import quaternion_matrix
from gazebo_ros_plugins_msgs.msg import FloatStamped


class FinModel(object):
    def __init__(self, index, pos, quat, topic):
        self.id = index
        self.pos = pos
        self.quat = quat
        self.topic = topic
        self.rot = quaternion_matrix(quat)[0:3, 0:3]

        unit_z = self.rot[:, 2]
        # Surge velocity wrt vehicle's body frame
        surge_vel = np.array([1, 0, 0])
        fin_surge_vel = surge_vel - np.dot(surge_vel, unit_z) / np.linalg.norm(unit_z)**2 * unit_z        
        # Compute the lift and drag vectors
        self.lift_vector = -1 * np.cross(unit_z, fin_surge_vel) / np.linalg.norm(np.cross(unit_z, fin_surge_vel))
        self.drag_vector = -1 * surge_vel / np.linalg.norm(surge_vel)       

        self.pub = rospy.Publisher(self.topic, FloatStamped, queue_size=3)
    
    def publish_command(self, delta):
        msg = FloatStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = delta
        self.pub.publish(msg)
