import rospy
import numpy as np
from copy import deepcopy
from os.path import isfile
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from control_msgs.srv import *

from control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import trajectory_generator
import waypoints
import logging
import sys
import tf2_ros
import time
from threading import Lock, Event
from tf.transformations import quaternion_about_axis, quaternion_multiply, \
    quaternion_inverse, quaternion_matrix, euler_from_quaternion


class DPControllerPathRandomFile(object):
    """
    Local planner for the dynamic positioning controllers to interpolate
    trajectories and generate trajectories from interpolated waypoint paths.
    """

    def __init__(self, full_dof=False, stamped_pose_only=False, thrusters_only=True):
        self._logger = logging.getLogger('dp_local_planner')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter(
            rospy.get_namespace().replace('/', '').upper() + ' -- %(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        self._lock = Lock()

        self._traj_interpolator = trajectory_generator.TrajectoryGenerator(
            full_dof=full_dof, stamped_pose_only=stamped_pose_only)

        # Max. allowed forward speed
        self._max_forward_speed = rospy.get_param('~max_forward_speed', 1.0)

        self._idle_rect_origin = None
        self._idle_z = None

        self._logger.info('Max. forward speed [m/s]=%.2f' % self._max_forward_speed)

        # Is underactuated?
        self._is_underactuated = rospy.get_param('~is_underactuated', False)

        self.inertial_frame_id = 'world'
        self.transform_ned_to_enu = None
        self.q_ned_to_enu = None
        if rospy.has_param('~inertial_frame_id'):
            self.inertial_frame_id = rospy.get_param('~inertial_frame_id')
            assert len(self.inertial_frame_id) > 0
            assert self.inertial_frame_id in ['world', 'world_ned']

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)

        rospy.set_param('inertial_frame_id', self.inertial_frame_id)

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        tf_trans_ned_to_enu = None
        try:
            tf_trans_ned_to_enu = tf_buffer.lookup_transform(
                'world', 'world_ned', rospy.Time(),
                rospy.Duration(10))
        except Exception, e:
            tf_trans_ned_to_enu = None
            self._logger.error('No transform found between world and the '
                               'inertial_frame_id provided ' +
                               rospy.get_namespace())

        if tf_trans_ned_to_enu is not None:
            self.q_ned_to_enu = np.array(
                [tf_trans_ned_to_enu.transform.rotation.x,
                 tf_trans_ned_to_enu.transform.rotation.y,
                 tf_trans_ned_to_enu.transform.rotation.z,
                 tf_trans_ned_to_enu.transform.rotation.w])
            self.transform_ned_to_enu = quaternion_matrix(
                self.q_ned_to_enu)[0:3, 0:3]

            self._logger.info('Transform world_ned (NED) to world (ENU)=\n' +
                              str(self.transform_ned_to_enu))

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)
        self._logger.info('Max. forward speed = ' +
                          str(self._max_forward_speed))

        for method in self._traj_interpolator.get_interpolator_tags():
            if rospy.has_param('~' + method):
                self._logger.info('Parameters for interpolation method <%s> found' % method)
                params = rospy.get_param('~' + method)
                self._logger.info('\t' + str(params))

                self._traj_interpolator.set_interpolator_parameters(method, params)
            else:
                self._logger.info('No parameters for interpolation method <%s> found' % method)

        self.init_odom_event = Event()
        self.init_odom_event.clear()

        self._timeout_idle_mode = rospy.get_param('~timeout_idle_mode', 5)
        self._start_count_idle = rospy.get_time()

        self._thrusters_only = thrusters_only

        if not self._thrusters_only:
            self._look_ahead_delay = rospy.get_param('~look_ahead_delay', 3.0)
        else:
            self._look_ahead_delay = 0.0

        self._station_keeping_center = None

        # Publishing topic for the trajectory given to the controller
        self._trajectory_pub = rospy.Publisher('trajectory',
                                               Trajectory,
                                               queue_size=1)
        # Publishing waypoints
        self._waypoints_pub = rospy.Publisher('waypoints',
                                               WaypointSet,
                                               queue_size=1)

        self._station_keeping_pub = rospy.Publisher('station_keeping_on',
                                                    Bool,
                                                    queue_size=1)

        self._automatic_control_pub = rospy.Publisher('automatic_on',
                                                      Bool,
                                                      queue_size=1)

        self._traj_tracking_pub = rospy.Publisher('trajectory_tracking_on',
                                                  Bool,
                                                  queue_size=1)

        self._interp_visual_markers = rospy.Publisher('interpolator_visual_markers',
                                                      MarkerArray,
                                                      queue_size=1)


        self._waypoints_msg = None
        self._trajectory_msg = None

        # Subscribing topic for the trajectory given to the controller
        self._input_trajectory_sub = rospy.Subscriber(
            'input_trajectory', Trajectory, self._update_trajectory_from_msg)

        self._max_time_pub = rospy.Publisher('time_to_target', Float64, queue_size=1)

        self._traj_info_update_timer = rospy.Timer(rospy.Duration(0.2),
            self._publish_trajectory_info)
        # Flag to activate station keeping
        self._station_keeping_on = True
        # Flag to set vehicle control to automatic
        self._is_automatic = True
        # Flag true if a trajectory is being tracked
        self._traj_running = False
        # Current vehicle pose
        self._vehicle_pose = None
        # Current reference point
        self._this_ref_pnt = None
        # Flag that indicates that a waypoint set has been initialized
        self._smooth_approach_on = False
        # Time stamp for received trajectory
        self._stamp_trajectory_received = 0.0
        # Dictionary of services
        self._services = dict()
        self._services['hold_vehicle'] = rospy.Service(
            'hold_vehicle', Hold, self.hold_vehicle)
        self._services['start_waypoint_list'] = rospy.Service(
            'start_waypoint_list', InitWaypointSet, self.start_waypoint_list)
        self._services['init_waypoints_from_file'] = rospy.Service(
            'init_waypoints_from_file', InitWaypointsFromFile,
            self.init_waypoints_from_file)
        self._services['go_to'] = rospy.Service('go_to', GoTo, self.go_to)
        self._services['go_to_incremental'] = rospy.Service(
            'go_to_incremental', GoToIncremental, self.go_to_incremental)

    def __del__(self):
        """Remove logging message handlers"""
        while self._logger.handlers:
            self._logger.handlers.pop()

    def _transform_position(self, vec, target, source):
        if target == source:
            return vec
        if target == 'world':
            return np.dot(self.transform_ned_to_enu, vec)
        if target == 'world_ned':
            return np.dot(self.transform_ned_to_enu.T, vec)

    def _transform_waypoint(self, waypoint):
        output = deepcopy(waypoint)
        output.pos = self._transform_position(output.pos,
                                              self.inertial_frame_id,
                                              output.inertial_frame_id)
        output.inertial_frame_id = self.inertial_frame_id
        output.max_forward_speed = min(waypoint.max_forward_speed, self._max_forward_speed)
        return output

    def _transform_waypoint_set(self, waypoint_set):
        output = waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = self._transform_waypoint(waypoint_set.get_waypoint(i))
            output.add_waypoint(wp)
        return output

    def _apply_workspace_constraints(self, waypoint_set):
        wp_set = waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = waypoint_set.get_waypoint(i)
            if wp.z > 0 and self.inertial_frame_id == 'world':
                continue
            if wp.z < 0 and self.inertial_frame_id == 'world_ned':
                continue
            wp_set.add_waypoint(wp)
        return wp_set

    def _publish_trajectory_info(self, event):
        """
        Publish messages for the waypoints, trajectory and internal flags.
        """
        if self._waypoints_msg is not None:
            self._waypoints_pub.publish(self._waypoints_msg)
        if self._trajectory_msg is not None:
            self._trajectory_pub.publish(self._trajectory_msg)
        markers = self._traj_interpolator.get_visual_markers()
        if markers is not None:
            self._interp_visual_markers.publish(markers)
        else:
            self._interp_visual_markers.publish(MarkerArray())
        self._station_keeping_pub.publish(Bool(self._station_keeping_on))
        self._automatic_control_pub.publish(Bool(self._is_automatic))
        self._traj_tracking_pub.publish(Bool(self._traj_running))
        return True

    def _update_trajectory_info(self):
        self._waypoints_msg = WaypointSet()
        if self._traj_interpolator.is_using_waypoints():
            wps = self._traj_interpolator.get_waypoints()
            if wps is not None:
                wps.inertial_frame_id = self.inertial_frame_id
                self._waypoints_msg = wps.to_message()
                self._waypoints_msg.header.frame_id = self.inertial_frame_id
        msg = self._traj_interpolator.get_trajectory_as_message()
        if msg is not None:
            msg.header.frame_id = self.inertial_frame_id
            self._trajectory_msg = msg
            self._logger.info('Updating the trajectory information')
        else:
            self._trajectory_msg = None
            self._logger.error('Error generating trajectory message')

    def _calc_smooth_approach(self):
        """
        Add the current vehicle position as waypoint to allow a smooth
        approach to the given trajectory.
        """
        if self._vehicle_pose is None:
            self._logger.error('Simulation not properly initialized yet, ignoring approach...')
            return
        if not self._traj_interpolator.is_using_waypoints():
            self._logger.error('Not using the waypoint interpolation method')
            return

        heading = euler_from_quaternion(self.get_vehicle_rot())[2]

        if self._thrusters_only:
            init_wp = waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],
                y=self._vehicle_pose.pos[1],
                z=self._vehicle_pose.pos[2],
                max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed,
                heading_offset=self._traj_interpolator.get_waypoints().get_waypoint(0).heading_offset)
        else:
            max_speed = self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed
            init_wp = waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],# + max_speed / self._look_ahead_delay * np.cos(heading),
                y=self._vehicle_pose.pos[1],# + max_speed / self._look_ahead_delay * np.sin(heading),
                z=self._vehicle_pose.pos[2],
                max_forward_speed=max_speed,
                heading_offset=self._traj_interpolator.get_waypoints().get_waypoint(0).heading_offset)
        first_wp = self._traj_interpolator.get_waypoints().get_waypoint(0)

        dx = first_wp.x - init_wp.x
        dy = first_wp.y - init_wp.y
        dz = first_wp.z - init_wp.z

        # One new waypoint at each meter
        self._logger.info('Adding waypoints to approach the first position in the given waypoint set')
        steps = int(np.floor(first_wp.dist(init_wp.pos)) / 10)
        if steps > 0 and self._traj_interpolator.get_interp_method() != 'dubins':
            for i in range(1, steps):
                wp = waypoints.Waypoint(
                    x=first_wp.x - i * dx / steps,
                    y=first_wp.y - i * dy / steps,
                    z=first_wp.z - i * dz / steps,
                    max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed)
                self._traj_interpolator.add_waypoint(wp, add_to_beginning=True)
        self._traj_interpolator.add_waypoint(init_wp, add_to_beginning=True)
        self._update_trajectory_info()

    def is_station_keeping_on(self):
        return self._station_keeping_on

    def is_automatic_on(self):
        return self._is_automatic

    def set_station_keeping(self, is_on=True):
        """Set station keeping mode flag."""
        self._station_keeping_on = is_on
        self._logger.info('STATION KEEPING MODE = ' + ('ON' if is_on else 'OFF'))

    def set_automatic_mode(self, is_on=True):
        """Set automatic mode flag."""
        self._is_automatic = is_on
        self._logger.info('AUTOMATIC MODE = ' + ('ON' if is_on else 'OFF'))

    def set_trajectory_running(self, is_on=True):
        """Set trajectory tracking flag."""
        self._traj_running = is_on
        self._logger.info('TRAJECTORY TRACKING = ' + ('ON' if is_on else 'OFF'))

    def has_started(self):
        """
        Return if the trajectory interpolator has started generating reference
        points.
        """

        return self._traj_interpolator.has_started()

    def has_finished(self):
        return self._traj_interpolator.has_finished()

    def update_vehicle_pose(self, pos, quat):
        if self._vehicle_pose is None:
            self._vehicle_pose = trajectory_generator.TrajectoryPoint()
        self._vehicle_pose.pos = pos
        self._vehicle_pose.rotq = quat
        self._vehicle_pose.t = rospy.get_time()
        self.init_odom_event.set()

    def get_vehicle_rot(self):
        self.init_odom_event.wait()
        return self._vehicle_pose.rotq

    def _update_trajectory_from_msg(self, msg):
        self._stamp_trajectory_received = rospy.get_time()
        self._traj_interpolator.init_from_trajectory_message(msg)
        self._logger.info('New trajectory received at ' + str(self._stamp_trajectory_received) + 's')
        self._update_trajectory_info()

    def start_station_keeping(self):
        if self._vehicle_pose is not None:
            self._this_ref_pnt = deepcopy(self._vehicle_pose)
            self._this_ref_pnt.vel = np.zeros(6)
            self._this_ref_pnt.acc = np.zeros(6)
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self._smooth_approach_on = False

    def hold_vehicle(self, request):
        """
        Service callback function to hold the vehicle's current position.
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose of the vehicle is invalid')
            return HoldResponse(False)
        self.start_station_keeping()
        return HoldResponse(True)

    def start_waypoint_list(self, request):
        """
        Service callback function to follow a set of waypoints
        Args:
            request (InitWaypointSet)
        """
        if len(request.waypoints) == 0:
            self._logger.error('Waypoint list is empty')
            return InitWaypointSetResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitWaypointSetResponse(False)
        else:
            self._logger.info('Start waypoint trajectory now!')
        self._lock.acquire()
        # Create a waypoint set
        wp_set = waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        # Create a waypoint set message, to fill wp_set
        waypointset_msg = WaypointSet()
        waypointset_msg.header.stamp = rospy.get_time()
        waypointset_msg.header.frame_id = self.inertial_frame_id
        if request.start_now:
            waypointset_msg.start_time = rospy.get_time()
        else:
            waypointset_msg.start_time = t.to_sec()
        waypointset_msg.waypoints = request.waypoints
        wp_set.from_message(waypointset_msg)
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_rect_origin = None
            self._smooth_approach_on = True
            self._logger.info('============================')
            self._logger.info('      WAYPOINT SET          ')
            self._logger.info('============================')
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting time = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            return InitWaypointSetResponse(True)
        else:
            self._logger.error('Error occurred while parsing waypoints')
            self._lock.release()
            return InitWaypointSetResponse(False)

    def init_waypoints_from_file(self, request):
        if (len(request.filename.data) == 0 or
                not isfile(request.filename.data)):
            self._logger.error('Invalid waypoint file')
            return InitWaypointsFromFileResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitWaypointsFromFileResponse(False)
        else:
            self._logger.info('Start waypoint trajectory now!')
        self._lock.acquire()
        self.set_station_keeping(True)
        self._traj_interpolator.set_interp_method(request.interpolator.data)

        wp_set = waypoints.WaypointSet()
        if not wp_set.read_from_file(request.filename.data):
            self._logger.info('Error occurred while parsing waypoint file')
            return InitWaypointsFromFileResponse(False)
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_rect_origin = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('IMPORT WAYPOINTS FROM FILE')
            self._logger.info('============================')
            self._logger.info('Filename = ' + request.filename.data)
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting time = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            return InitWaypointsFromFileResponse(True)
        else:
            self._logger.error('Error occurred while parsing waypoint file')
            self._lock.release()
            return InitWaypointsFromFileResponse(False)

    def go_to(self, request):
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToResponse(False)
        if request.waypoint.max_forward_speed <= 0.0:
            self._logger.error('Max. forward speed must be greater than zero')
            return GoToResponse(False)
        self.set_station_keeping(True)
        self._lock.acquire()
        wp_set = waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)

        init_wp = waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.waypoint.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            use_fixed_heading=request.waypoint.use_fixed_heading,
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(init_wp)
        wp_set.add_waypoint_from_msg(request.waypoint)
        wp_set = self._transform_waypoint_set(wp_set)
        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            return GoToResponse(False)

        self._station_keeping_center = None
        t = rospy.get_time()
        self._traj_interpolator.set_start_time(t)
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_rect_origin = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO')
        self._logger.info('============================')
        self._logger.info('Heading offset [rad] = %.2f' % request.waypoint.heading_offset)
        self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
        self._logger.info('Starting from = ' + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos))
        self._logger.info('Start time [s] = %.2f ' % t)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        return GoToResponse(True)

    def go_to_incremental(self, request):
        """
        Service callback to set the command to the vehicle to move to a
        relative position in the world.
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToIncrementalResponse(False)
        if request.max_forward_speed <= 0:
            self._logger.error('Max. forward speed must be positive')
            return GoToIncrementalResponse(False)

        self._lock.acquire()
        self.set_station_keeping(True)
        wp_set = waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        init_wp = waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(init_wp)

        wp = waypoints.Waypoint(
            x=self._vehicle_pose.pos[0] + request.step.x,
            y=self._vehicle_pose.pos[1] + request.step.y,
            z=self._vehicle_pose.pos[2] + request.step.z,
            max_forward_speed=request.max_forward_speed,
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(wp)

        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            return GoToIncrementalResponse(False)

        self._station_keeping_center = None
        self._traj_interpolator.set_start_time(rospy.Time.now().to_sec())
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_rect_origin = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO INCREMENTAL')
        self._logger.info('============================')
        self._logger.info(str(wp_set))
        self._logger.info('# waypoints = %d' % wp_set.num_waypoints)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        return GoToIncrementalResponse(True)

    def generate_reference(self, t):
        pnt = self._traj_interpolator.generate_reference(t, self._vehicle_pose.pos, self.get_vehicle_rot())
        if pnt is None:
            return self._vehicle_pose
        else:
            return pnt

    def interpolate(self, t):
        """
        Function interface to the controller. Calls the interpolator to
        calculate the current trajectory sample or returns a fixed position
        based on the past odometry measurements for station keeping.
        """

        self._lock.acquire()
        if not self._station_keeping_on and self._traj_running:
            if self._smooth_approach_on:
                # Generate extra waypoint before the initial waypoint
                self._calc_smooth_approach()
                self._smooth_approach_on = False
                self._update_trajectory_info()
                time.sleep(0.0)
                self._logger.info('Adding waypoints to approach the given waypoint trajectory')

            # Get interpolated reference from the reference trajectory
            self._this_ref_pnt = self._traj_interpolator.interpolate(t, self._vehicle_pose.pos, self.get_vehicle_rot())

            if self._look_ahead_delay > 0:
                self._this_ref_pnt = self.generate_reference(t + self._look_ahead_delay)

            self._max_time_pub.publish(Float64(self._traj_interpolator.get_max_time() - rospy.get_time()))

            if not self._traj_running:
                self._traj_running = True
                self._logger.info(rospy.get_namespace() + ' - Trajectory running')

            if self._traj_running and (self._traj_interpolator.has_finished() or self._station_keeping_on):
                # Trajectory ended,
                self._logger.info(rospy.get_namespace() + ' - Trajectory completed!')

                self._this_ref_pnt.vel = np.zeros(6)
                self._this_ref_pnt.acc = np.zeros(6)
                self._start_count_idle = rospy.get_time()
                self.set_station_keeping(True)
                self.set_automatic_mode(False)
                self.set_trajectory_running(False)
                self._update_trajectory_info()
        elif self._this_ref_pnt is None:
            self._traj_interpolator.set_interp_method('lipb')
            # Use the latest position and heading of the vehicle from the odometry to enter station keeping mode
            self._this_ref_pnt = deepcopy(self._vehicle_pose)
            # Set roll and pitch reference to zero
            yaw = self._this_ref_pnt.rot[2]
            self._this_ref_pnt.rot = [0, 0, yaw]
            self.set_automatic_mode(False)
        elif self._station_keeping_on:
            self._max_time_pub.publish(Float64(0))
            #######################################################################
            if not self._thrusters_only and rospy.get_time() - self._start_count_idle > self._timeout_idle_mode:
                self._logger.info('AUV STATION KEEPING')
                if self._station_keeping_center is None:
                    self._station_keeping_center = self._this_ref_pnt

                wp_set = self._traj_interpolator.get_waypoints()
                wp_set = self._apply_workspace_constraints(wp_set)
                if wp_set.is_empty:
                    raise rospy.ROSException('Waypoints violate workspace constraints, are you using world or world_ned as reference?')

                # Activates station keeping
                self.set_station_keeping(True)
                self._traj_interpolator.set_interp_method('cubic')
                self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot())
                self._traj_interpolator.set_start_time(rospy.get_time())
                self._update_trajectory_info()
                # Disables station keeping to start trajectory
                self.set_station_keeping(False)
                self.set_automatic_mode(True)
                self.set_trajectory_running(True)
                self._smooth_approach_on = True
            #######################################################################"""
        self._lock.release()
        return self._this_ref_pnt
