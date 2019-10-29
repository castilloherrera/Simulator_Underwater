#!/usr/bin/env python
import math
import numpy as np
import rospy
from copy import deepcopy

import geometry_msgs.msg as geometry_msgs
import control_msgs.msg as control_msgs
import tf
import tf.transformations as trans

from nav_msgs.msg import Odometry
from thrusters.models import Thruster
from gazebo_ros_plugins_msgs.msg import FloatStamped
from control_msgs.msg import TrajectoryPoint
from control_interfaces import DPControllerLocalPlanner
import tf2_ros
from tf.transformations import quaternion_matrix

class AUVGeometricTrackingControllerBluerov2Circle:
    def __init__(self):
        self.namespace = rospy.get_namespace().replace('/', '')
        rospy.loginfo('Initialize control for vehicle <%s>' % self.namespace)

        self.local_planner = DPControllerLocalPlanner(full_dof=True, thrusters_only=False,
            stamped_pose_only=False)

        self.base_link = rospy.get_param('~base_link', 'base_link')

        # Reading the minimum thrust generated
        self.min_thrust = rospy.get_param('~min_thrust', 0)
        assert self.min_thrust >= 0
        rospy.loginfo('Min. thrust [N]=%.2f', self.min_thrust)

        # Reading the maximum thrust generated
        self.max_thrust = rospy.get_param('~max_thrust', 0)
        assert self.max_thrust > 0 and self.max_thrust > self.min_thrust
        rospy.loginfo('Max. thrust [N]=%.2f', self.max_thrust)

        # Reading the thruster topic
        self.thruster_topic_0 = rospy.get_param('~thruster_topic_0', 'thrusters/0/input')
        assert len(self.thruster_topic_0) > 0

        # Reading the thruster topic
        self.thruster_topic_1 = rospy.get_param('~thruster_topic_1', 'thrusters/1/input')
        assert len(self.thruster_topic_1) > 0

        # Reading the thruster topic
        self.thruster_topic_2 = rospy.get_param('~thruster_topic_2', 'thrusters/2/input')
        assert len(self.thruster_topic_2) > 0

        # Reading the thruster topic
        self.thruster_topic_3 = rospy.get_param('~thruster_topic_3', 'thrusters/3/input')
        assert len(self.thruster_topic_3) > 0

        # Reading the thruster topic
        #self.thruster_topic_4 = rospy.get_param('~thruster_topic_4', 'thrusters/4/input')
        #assert len(self.thruster_topic_4) > 0

        # Reading the thruster topic
        #self.thruster_topic_5 = rospy.get_param('~thruster_topic_5', 'thrusters/5/input')
        #assert len(self.thruster_topic_5) > 0

        # Reading the thruster gain
        self.p_gain_thrust = rospy.get_param('~thrust_p_gain', 0.0)
        assert self.p_gain_thrust > 0

        self.d_gain_thrust = rospy.get_param('~thrust_d_gain', 0.0)
        assert self.d_gain_thrust >= 0

        # Reading the roll gain
        self.p_roll = rospy.get_param('~p_roll', 0.0)
        assert self.p_roll > 0

        # Reading the pitch P gain
        self.p_pitch = rospy.get_param('~p_pitch', 0.0)
        assert self.p_pitch > 0

        # Reading the pitch D gain
        self.d_pitch = rospy.get_param('~d_pitch', 0.0)
        assert self.d_pitch >= 0

        # Reading the yaw P gain
        self.p_yaw = rospy.get_param('~p_yaw', 0.0)
        assert self.p_yaw > 0

        # Reading the yaw D gain
        self.d_yaw = rospy.get_param('~d_yaw', 0.0)
        assert self.d_yaw >= 0

        # Reading the saturation for the desired pitch
        self.desired_pitch_limit = rospy.get_param('~desired_pitch_limit', 15 * np.pi / 180)
        assert self.desired_pitch_limit > 0

        # Reading the saturation for yaw error
        self.yaw_error_limit = rospy.get_param('~yaw_error_limit', 1.0)
        assert self.yaw_error_limit > 0

        # Retrieve the thruster configuration parameters
        self.thruster_config = rospy.get_param('~thruster_config')

        # Check if all necessary thruster model parameter are available
        thruster_params = ['conversion_fcn_params', 'conversion_fcn',
            'topic_prefix', 'topic_suffix', 'frame_base', 'max_thrust']
        for p in thruster_params:
            if p not in self.thruster_config:
                raise rospy.ROSException(
                    'Parameter <%s> for thruster conversion function is '
                    'missing' % p)

# Truster 0 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        self.thruster_topic_0 = '/%s/%s/%d/%s' %  (self.namespace,
            self.thruster_config['topic_prefix'], 0,
            self.thruster_config['topic_suffix'])

        base_0 = '%s/%s' % (self.namespace, self.base_link)

        frame_0 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.tf_buffer_0 = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer_0)

        rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_0, frame_0))
        trans_0 = self.tf_buffer_0.lookup_transform(base_0, frame_0, rospy.Time(), rospy.Duration(5))
        pos_0 = np.array([trans_0.transform.translation.x,
                        trans_0.transform.translation.y,
                        trans_0.transform.translation.z])
        quat_0 = np.array([trans_0.transform.rotation.x,
                         trans_0.transform.rotation.y,
                         trans_0.transform.rotation.z,
                         trans_0.transform.rotation.w])
        rospy.loginfo('Thruster transform found %s -> %s' % (base_0, frame_0))
        rospy.loginfo('pos=' + str(pos_0))
        rospy.loginfo('rot=' + str(quat_0))

        # Read transformation from thruster
        self.thruster_0 = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 0,
            self.thruster_topic_0, pos_0, quat_0,
            **self.thruster_config['conversion_fcn_params'])

        rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        rospy.loginfo('Thruster input topic=' + self.thruster_topic_0)

# Truster 1 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        self.thruster_topic_1 = '/%s/%s/%d/%s' %  (self.namespace,
            self.thruster_config['topic_prefix'], 1,
            self.thruster_config['topic_suffix'])

        base_1 = '%s/%s' % (self.namespace, self.base_link)

        frame_1 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.tf_buffer_1 = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer_1)

        rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_1, frame_1))
        trans_1 = self.tf_buffer_1.lookup_transform(base_1, frame_1, rospy.Time(), rospy.Duration(5))
        pos_1 = np.array([trans_1.transform.translation.x,
                        trans_1.transform.translation.y,
                        trans_1.transform.translation.z])
        quat_1 = np.array([trans_1.transform.rotation.x,
                         trans_1.transform.rotation.y,
                         trans_1.transform.rotation.z,
                         trans_1.transform.rotation.w])
        rospy.loginfo('Thruster transform found %s -> %s' % (base_1, frame_1))
        rospy.loginfo('pos=' + str(pos_1))
        rospy.loginfo('rot=' + str(quat_1))

        # Read transformation from thruster
        self.thruster_1 = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 1,
            self.thruster_topic_1, pos_1, quat_1,
            **self.thruster_config['conversion_fcn_params'])

        rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        rospy.loginfo('Thruster input topic=' + self.thruster_topic_1)

# Truster 2 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        self.thruster_topic_2 = '/%s/%s/%d/%s' %  (self.namespace,
            self.thruster_config['topic_prefix'], 2,
            self.thruster_config['topic_suffix'])

        base_2 = '%s/%s' % (self.namespace, self.base_link)

        frame_2 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.tf_buffer_2 = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer_2)

        rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_2, frame_2))
        trans_2 = self.tf_buffer_2.lookup_transform(base_2, frame_2, rospy.Time(), rospy.Duration(5))
        pos_2 = np.array([trans_2.transform.translation.x,
                        trans_2.transform.translation.y,
                        trans_2.transform.translation.z])
        quat_2 = np.array([trans_2.transform.rotation.x,
                         trans_2.transform.rotation.y,
                         trans_2.transform.rotation.z,
                         trans_2.transform.rotation.w])
        rospy.loginfo('Thruster transform found %s -> %s' % (base_2, frame_2))
        rospy.loginfo('pos=' + str(pos_2))
        rospy.loginfo('rot=' + str(quat_2))

        # Read transformation from thruster
        self.thruster_2 = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 2,
            self.thruster_topic_2, pos_2, quat_2,
            **self.thruster_config['conversion_fcn_params'])

        rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        rospy.loginfo('Thruster input topic=' + self.thruster_topic_2)

# Truster 3 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        self.thruster_topic_3 = '/%s/%s/%d/%s' %  (self.namespace,
            self.thruster_config['topic_prefix'], 3,
            self.thruster_config['topic_suffix'])

        base_3 = '%s/%s' % (self.namespace, self.base_link)

        frame_3 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.tf_buffer_3 = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer_3)

        rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_3, frame_3))
        trans_3 = self.tf_buffer_3.lookup_transform(base_3, frame_3, rospy.Time(), rospy.Duration(5))
        pos_3 = np.array([trans_3.transform.translation.x,
                        trans_3.transform.translation.y,
                        trans_3.transform.translation.z])
        quat_3 = np.array([trans_3.transform.rotation.x,
                         trans_3.transform.rotation.y,
                         trans_3.transform.rotation.z,
                         trans_3.transform.rotation.w])
        rospy.loginfo('Thruster transform found %s -> %s' % (base_3, frame_3))
        rospy.loginfo('pos=' + str(pos_3))
        rospy.loginfo('rot=' + str(quat_3))

        # Read transformation from thruster
        self.thruster_3 = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 3,
            self.thruster_topic_3, pos_3, quat_3,
            **self.thruster_config['conversion_fcn_params'])

        rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        rospy.loginfo('Thruster input topic=' + self.thruster_topic_3)

# Truster 4 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        #self.thruster_topic_4 = '/%s/%s/%d/%s' %  (self.namespace,
        #    self.thruster_config['topic_prefix'], 4,
        #    self.thruster_config['topic_suffix'])

        #base_4 = '%s/%s' % (self.namespace, self.base_link)

        #frame_4 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 4)

        #self.tf_buffer_4 = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tf_buffer_4)

        #rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_4, frame_4))
        #trans_4 = self.tf_buffer_4.lookup_transform(base_4, frame_4, rospy.Time(), rospy.Duration(5))
        #pos_4 = np.array([trans_4.transform.translation.x,
        #                trans_4.transform.translation.y,
        #                trans_4.transform.translation.z])
        #quat_4 = np.array([trans_4.transform.rotation.x,
        #                 trans_4.transform.rotation.y,
        #                 trans_4.transform.rotation.z,
        #                 trans_4.transform.rotation.w])
        #rospy.loginfo('Thruster transform found %s -> %s' % (base_4, frame_4))
        #rospy.loginfo('pos=' + str(pos_4))
        #rospy.loginfo('rot=' + str(quat_4))

        # Read transformation from thruster
        #self.thruster_4 = Thruster.create_thruster(
        #    self.thruster_config['conversion_fcn'], 4,
        #    self.thruster_topic_4, pos_4, quat_4,
        #    **self.thruster_config['conversion_fcn_params'])

        #rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        #rospy.loginfo('Thruster input topic=' + self.thruster_topic_4)

# Truster 5 --------------------------------------------------------------------------------------------------------
        # Setting up the thruster topic name
        #self.thruster_topic_5 = '/%s/%s/%d/%s' %  (self.namespace,
        #    self.thruster_config['topic_prefix'], 5,
        #    self.thruster_config['topic_suffix'])

        #base_5 = '%s/%s' % (self.namespace, self.base_link)

        #frame_5 = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 5)

        #self.tf_buffer_5 = tf2_ros.Buffer()
        #self.listener = tf2_ros.TransformListener(self.tf_buffer_5)

        #rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base_5, frame_5))
        #trans_5 = self.tf_buffer_5.lookup_transform(base_5, frame_5, rospy.Time(), rospy.Duration(5))
        #pos_5 = np.array([trans_5.transform.translation.x,
        #                trans_5.transform.translation.y,
        #                trans_5.transform.translation.z])
        #quat_5 = np.array([trans_5.transform.rotation.x,
        #                 trans_5.transform.rotation.y,
        #                 trans_5.transform.rotation.z,
        #                 trans_5.transform.rotation.w])
        #rospy.loginfo('Thruster transform found %s -> %s' % (base_5, frame_5))
        #rospy.loginfo('pos=' + str(pos_5))
        #rospy.loginfo('rot=' + str(quat_5))

        # Read transformation from thruster
        #self.thruster_5 = Thruster.create_thruster(
        #    self.thruster_config['conversion_fcn'], 5,
        #    self.thruster_topic_5, pos_5, quat_5,
        #    **self.thruster_config['conversion_fcn_params'])

        #rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        #rospy.loginfo('Thruster input topic=' + self.thruster_topic_5)

# --------------------------------------------------------------------------------------------------------
        self.odometry_sub = rospy.Subscriber(
            'odom', Odometry, self.odometry_callback)

        self.reference_pub = rospy.Publisher(
            'reference', TrajectoryPoint, queue_size=1)

        # Publish error (for debugging)
        self.error_pub = rospy.Publisher(
            'error', TrajectoryPoint, queue_size=1)

    @staticmethod
    def unwrap_angle(t):
        return math.atan2(math.sin(t),math.cos(t))

    @staticmethod
    def vector_to_np(v):
        return np.array([v.x, v.y, v.z])

    @staticmethod
    def quaternion_to_np(q):
        return np.array([q.x, q.y, q.z, q.w])

    def odometry_callback(self, msg):
        """Handle odometry callback: The actual control loop."""

        # Update local planner's vehicle position and orientation
        pos = [msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z]

        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]

        self.local_planner.update_vehicle_pose(pos, quat)

        # Compute the desired position
        t = rospy.Time.now().to_sec()
        des = self.local_planner.interpolate(t)

        # Publish the reference
        ref_msg = TrajectoryPoint()
        ref_msg.header.stamp = rospy.Time.now()
        ref_msg.header.frame_id = self.local_planner.inertial_frame_id
        ref_msg.pose.position = geometry_msgs.Vector3(*des.p)
        ref_msg.pose.orientation = geometry_msgs.Quaternion(*des.q)
        ref_msg.velocity.linear = geometry_msgs.Vector3(*des.vel[0:3])
        ref_msg.velocity.angular = geometry_msgs.Vector3(*des.vel[3::])

        self.reference_pub.publish(ref_msg)

        p = self.vector_to_np(msg.pose.pose.position)
        forward_vel = self.vector_to_np(msg.twist.twist.linear)
        ref_vel = des.vel[0:3]

        q = self.quaternion_to_np(msg.pose.pose.orientation)
        rpy = trans.euler_from_quaternion(q, axes='sxyz')

        # Compute tracking errors wrt world frame:
        e_p = des.p - p
        abs_pos_error = np.linalg.norm(e_p)
        abs_vel_error = np.linalg.norm(ref_vel - forward_vel)

        # Generate error message
        error_msg = TrajectoryPoint()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = self.local_planner.inertial_frame_id
        error_msg.pose.position = geometry_msgs.Vector3(*e_p)
        error_msg.pose.orientation = geometry_msgs.Quaternion(
            *trans.quaternion_multiply(trans.quaternion_inverse(q), des.q))
        error_msg.velocity.linear = geometry_msgs.Vector3(*(des.vel[0:3] - self.vector_to_np(msg.twist.twist.linear)))
        error_msg.velocity.angular = geometry_msgs.Vector3(*(des.vel[3::] - self.vector_to_np(msg.twist.twist.angular)))

        # Based on position tracking error: Compute desired orientation
        pitch_des = -math.atan2(e_p[2], np.linalg.norm(e_p[0:2]))
        # Limit desired pitch angle:
        pitch_des = max(-self.desired_pitch_limit, min(pitch_des, self.desired_pitch_limit))

        yaw_des = math.atan2(e_p[1], e_p[0])
        yaw_err = self.unwrap_angle(yaw_des - rpy[2])

        # Limit yaw effort
        yaw_err = min(self.yaw_error_limit, max(-self.yaw_error_limit, yaw_err))

        # Roll: P controller to keep roll == 0
        roll_control = self.p_roll * rpy[0]

        # Pitch: P controller to reach desired pitch angle
        pitch_control = self.p_pitch * self.unwrap_angle(pitch_des - rpy[1]) + self.d_pitch * (des.vel[4] - msg.twist.twist.angular.y)

        # Yaw: P controller to reach desired yaw angle
        yaw_control = self.p_yaw * yaw_err + self.d_yaw * (des.vel[5] - msg.twist.twist.angular.z)

        rpy = np.array([roll_control, pitch_control, yaw_control])

        # Limit thrust
        thrust_0 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust_0 = max(self.min_thrust, thrust_0)

        thrust_force_0 = self.thruster_0.tam_column * thrust_0
        self.thruster_0.publish_command(thrust_force_0[0])
        self.error_pub.publish(error_msg)

        # Limit thrust
        thrust_1 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust_1 = max(self.min_thrust, thrust_1)

        thrust_force_1 = self.thruster_1.tam_column * thrust_1
        self.thruster_1.publish_command(thrust_force_1[0])
        self.error_pub.publish(error_msg)

        # Limit thrust
        thrust_2 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust_2 = max(self.min_thrust, thrust_2)

        thrust_force_2 = self.thruster_2.tam_column * thrust_2
        self.thruster_2.publish_command(thrust_force_2[0])
        self.error_pub.publish(error_msg)

        # Limit thrust
        thrust_3 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust_3 = max(self.min_thrust, thrust_3)

        thrust_force_3 = self.thruster_3.tam_column * thrust_3
        self.thruster_3.publish_command(thrust_force_3[0])
        self.error_pub.publish(error_msg)

        # Limit thrust
        #thrust_4 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        #thrust_4 = max(self.min_thrust, thrust_4)

        #thrust_force_4 = self.thruster_4.tam_column * thrust_4
        #self.thruster_4.publish_command(thrust_force_4[0])
        #self.error_pub.publish(error_msg)

        # Limit thrust
        #thrust_5 = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        #thrust_5 = max(self.min_thrust, thrust_5)

        #thrust_force_5 = self.thruster_5.tam_column * thrust_5
        #self.thruster_5.publish_command(thrust_force_5[0])
        #self.error_pub.publish(error_msg)

if __name__ == '__main__':
    print('Starting AUV trajectory tracker')
    rospy.init_node('auv_geometric_tracking_controller_bluerov2_circle')

    try:
        node = AUVGeometricTrackingControllerBluerov2Circle()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
