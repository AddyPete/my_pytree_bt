import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import numpy as np
import time
from std_msgs.msg import Bool, String, Float32, Int32
from py_trees.common import Status
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


BASE_WAYPOINT_OBJECTS = [
    [3.8, 0],
    [3.8, 2.0],
    [0.6, 2.0],
]
# BASE_WAYPOINT_OBJECTS = [
#     [10.0, 0],
#     [10.3, 2.2],
#     [-0.7, 2.2],
#     [-0.7, 41.5],
# ]  # RAMP PHYSICS

DIFF_ANGLE_THRESHOLD = 6  # DEGREE
DISTANCE_TO_TARGET_THRESHOLD = 0.6  # meters
ARUCO_FINISHED_MISISON_THRESH = 0.6  # meters
ARUCO_NEAR_THRESH = 0.5

ANGULAR_VEL = 0.05
ROTATION_VEL = 0.2
FLICK_ANGLE = 0.10
REVERSE_FLICK_FOR_STOP = -0.2

LINEAR_VEL_DICT = {"Incline": 0.60, "Flat": 0.23, "Decline": -0.1}
ROVER_ACTION_DICT = {
    "Left Steer": ANGULAR_VEL,
    "Right Steer": -ANGULAR_VEL,
    "No Steer": 0.0,
}
ROVER_ROTATION_DICT = {
    "Left Steer": -ROTATION_VEL,
    "Right Steer": ROTATION_VEL,
    "No Steer": 0.0,
}

PERIOD_MS = 50  # MS
TREE_DEBUG = True

TOPIC_QUEUE_SIZE = 10

INCLINE_WAIT = 0.25  # SEC
DECLINE_WAIT = 0.5
FLICK_STOP = 0.5
STOP = 1.0


class WaypointManager:
    def __init__(self, waypoints):
        self.waypoints = [(point, False) for point in waypoints]

    def update_waypoint_status(self, index, status):
        self.waypoints[index] = (self.waypoints[index][0], status)

    def is_waypoint_visited(self, index):
        return self.waypoints[index][1]

    def get_waypoint(self, index):
        return self.waypoints[index][0]

    def get_all_waypoints_with_status(self):
        return self.waypoints


class SensorData:
    def __init__(self):
        self.aruco_detected_id = self.aruco_detected_distance = -1

        ##### ZED VARIABLES
        self.zed_x_pos = self.zed_y_pos = self.zed_z_pos = 0.0
        self.zed_x_ori = self.zed_y_ori = self.zed_z_ori = self.zed_w_ori = 0.0
        self.zed_is_near_front_obs = self.zed_is_near_left_obs = (
            self.zed_is_near_right_obs
        ) = False
        self.zed_is_object_detected = False
        self.zed_slope = "Flat"

        ##### LIDAR VARIABLES
        self.lidar_is_near_front_wall = self.lidar_is_near_left_wall = (
            self.lidar_is_near_right_wall
        ) = False


class SensorSubscribers(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_data):
        super(SensorSubscribers, self).__init__(name)

        self.sensor_data = sensor_data

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        ##### ARUCO VARIABLES
        self.aruco_detected_id = self.aruco_detected_distance = -1

        ##### ZED VARIABLES
        self.zed_x_pos = self.zed_y_pos = self.zed_z_pos = 0.0
        self.zed_x_ori = self.zed_y_ori = self.zed_z_ori = self.zed_w_ori = 0.0
        self.zed_is_near_front_obs = self.zed_is_near_left_obs = (
            self.zed_is_near_right_obs
        ) = False
        self.zed_is_object_detected = False
        self.zed_slope = "Flat"

        ##### LIDAR VARIABLES
        self.lidar_is_near_front_wall = self.lidar_is_near_left_wall = (
            self.lidar_is_near_right_wall
        ) = False

        ##### ARUCO SUBSCRIBERS
        self.aruco_dist_subscriber = self.node.create_subscription(
            Float32,
            "/aruco/distance",
            self.aruco_dist_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

        self.aruco_id_subscriber = self.node.create_subscription(
            Int32, "/aruco/id", self.aruco_id_subscriber_callback, TOPIC_QUEUE_SIZE
        )

        ##### ZED SUBSCRIBERS
        self.zed_odom_subscriber = self.node.create_subscription(
            Odometry,
            "/zed/zed_node/odom",
            self.zed_odom_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )
        self.zed_front_collision_subscriber = self.node.create_subscription(
            Bool,
            "/zed/is_near_front_obs",
            self.zed_front_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

        self.zed_left_collision_subscriber = self.node.create_subscription(
            Bool,
            "/zed/is_near_left_obs",
            self.zed_left_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

        self.zed_right_collision_subscriber = self.node.create_subscription(
            Bool,
            "/zed/is_near_right_obs",
            self.zed_right_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )
        self.zed_slope_subscriber = self.node.create_subscription(
            String, "/zed/slope", self.zed_slope_subscriber_callback, TOPIC_QUEUE_SIZE
        )
        self.zed_object_detection_subscriber = self.node.create_subscription(
            Bool,
            "/zed/is_object_detected",
            self.zed_object_detection_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

        ##### LIDAR SUBSCRIBERS
        self.lidar_front_wall_collision_subscriber = self.node.create_subscription(
            Bool,
            "/lidar/is_near_front_wall",
            self.lidar_front_wall_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )
        self.lidar_left_wall_collision_subscriber = self.node.create_subscription(
            Bool,
            "/lidar/is_near_left_wall",
            self.lidar_left_wall_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

        self.lidar_right_wall_collision_subscriber = self.node.create_subscription(
            Bool,
            "/lidar/is_near_right_wall",
            self.lidar_right_wall_collision_subscriber_callback,
            TOPIC_QUEUE_SIZE,
        )

    def aruco_dist_subscriber_callback(self, msg):
        self.aruco_detected_distance = msg.data

    def aruco_id_subscriber_callback(self, msg):
        self.aruco_detected_id = msg.data

    def zed_odom_subscriber_callback(self, msg):
        self.zed_x_pos = msg.pose.pose.position.x
        self.zed_y_pos = msg.pose.pose.position.y
        self.zed_z_pos = msg.pose.pose.position.z
        self.zed_x_ori = msg.pose.pose.orientation.x
        self.zed_y_ori = msg.pose.pose.orientation.y
        self.zed_z_ori = msg.pose.pose.orientation.z
        self.zed_w_ori = msg.pose.pose.orientation.w

    def zed_front_collision_subscriber_callback(self, msg):
        self.zed_is_near_front_obs = msg.data

    def zed_left_collision_subscriber_callback(self, msg):
        self.zed_is_near_left_obs = msg.data

    def zed_right_collision_subscriber_callback(self, msg):
        self.zed_is_near_right_obs = msg.data

    def zed_slope_subscriber_callback(self, msg):
        self.zed_slope = msg.data

    def zed_object_detection_subscriber_callback(self, msg):
        self.zed_is_object_detected = msg.data

    def lidar_front_wall_collision_subscriber_callback(self, msg):
        self.lidar_is_near_front_wall = msg.data

    def lidar_left_wall_collision_subscriber_callback(self, msg):
        self.lidar_is_near_left_wall = msg.data

    def lidar_right_wall_collision_subscriber_callback(self, msg):
        self.lidar_is_near_right_wall = msg.data

    def update(self):

        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        self.sensor_data.aruco_detected_id = self.aruco_detected_id
        self.sensor_data.aruco_detected_distance = self.aruco_detected_distance

        self.sensor_data.zed_x_pos = self.zed_x_pos
        self.sensor_data.zed_y_pos = self.zed_y_pos
        self.sensor_data.zed_z_pos = self.zed_z_pos

        self.sensor_data.zed_x_ori = self.zed_x_ori
        self.sensor_data.zed_y_ori = self.zed_y_ori
        self.sensor_data.zed_z_ori = self.zed_z_ori
        self.sensor_data.zed_w_ori = self.zed_w_ori

        self.sensor_data.zed_is_near_front_obs = self.zed_is_near_front_obs
        self.sensor_data.zed_is_near_left_obs = self.zed_is_near_left_obs
        self.sensor_data.zed_is_near_right_obs = self.zed_is_near_right_obs

        self.sensor_data.zed_is_object_detected = self.zed_is_object_detected
        self.sensor_data.zed_slope = self.zed_slope

        self.sensor_data.lidar_is_near_front_wall = self.lidar_is_near_front_wall
        self.sensor_data.lidar_is_near_left_wall = self.lidar_is_near_left_wall
        self.sensor_data.idar_is_near_right_wall = self.lidar_is_near_right_wall

        return Status.RUNNING


class CheckDetectedObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_data):
        super(CheckDetectedObject, self).__init__(name)
        self.sensor_data = sensor_data

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, "/cmd_vel", TOPIC_QUEUE_SIZE
        )
        self.cmd_vel_msg = Twist()

    def update(self):
        zed_is_object_detected = self.sensor_data.zed_is_object_detected
        zed_slope = self.sensor_data.zed_slope

        self.node.get_logger().info(f"Object Detected: {zed_is_object_detected}")

        if zed_is_object_detected and zed_slope == "Flat":
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return Status.FAILURE
        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class ActionPolicy:
    def __init__(self, waypoint):
        self.waypoint = waypoint

        self.finished_mission = False

    def get_action_policy(self, x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori):
        # Constants
        # DIFF_ANGLE_THRESHOLD = 1  # Degree
        # DISTANCE_TO_TARGET_THRESHOLD = 1.0  # Meters

        # Calculate yaw
        yaw = np.degrees(
            np.arctan2(
                2 * (x_ori * y_ori + w_ori * z_ori), 1 - 2 * (y_ori**2 + z_ori**2)
            )
        )

        # Calculate target angle
        relDisX = self.waypoint[0] - x_pos
        relDisY = self.waypoint[1] - y_pos
        target_angle = np.degrees(np.arctan2(relDisY, relDisX))

        # Calculate angle difference and normalize it
        diff_angle = target_angle - yaw
        diff_angle = (diff_angle + 180) % 360 - 180  # Normalize to [-180, 180]

        # Decide rover action based on the difference in angle
        if abs(diff_angle) > DIFF_ANGLE_THRESHOLD:
            rover_action = "Left Steer" if diff_angle > 0 else "Right Steer"
        else:
            rover_action = "No Steer"

        # Calculate distance to the target
        distance_to_target = np.sqrt(relDisX**2 + relDisY**2)

        # Check if mission is finished
        if distance_to_target < DISTANCE_TO_TARGET_THRESHOLD:
            self.finished_mission = True
        else:
            self.finished_mission = False

        # Debug print to see the computed values
        # print(f"Distance: {distance_to_target:.2f} m, Action: {rover_action}, Finished: {self.finished_mission}")

        return rover_action, self.finished_mission, distance_to_target, diff_angle


class GoToWayPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint_manager, waypoint_index, sensor_data):
        super(GoToWayPoint, self).__init__(name)
        self.waypoint_index = waypoint_index
        self.waypoint_manager = waypoint_manager

        self.sensor_data = sensor_data

        self.current_waypoint = self.waypoint_manager.get_waypoint(waypoint_index)
        self.action_policy = ActionPolicy(self.current_waypoint)
        self.waypoint_finished_mission = False
        self.previous_zed_slope = None

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_msg = Twist()
        self.waypoint_status_publisher = self.node.create_publisher(
            String, "waypoint_status", TOPIC_QUEUE_SIZE
        )

        self.angle_requested_go_right_publisher = self.node.create_publisher(
            Bool, "angle_requested_go_right", TOPIC_QUEUE_SIZE
        )
        self.angle_requested_go_left_publisher = self.node.create_publisher(
            Bool, "angle_requested_go_left", TOPIC_QUEUE_SIZE
        )
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, "/cmd_vel", TOPIC_QUEUE_SIZE
        )
        self.distance_to_target_publisher = self.node.create_publisher(
            Float32, "/distance_to_target", 10
        )
        self.current_waypoint_publisher = self.node.create_publisher(
            Int32, "/current_waypoint", 10
        )
        self.current_waypoint_status_publisher = self.node.create_publisher(
            Bool, "/current_waypoint_status", 10
        )

    def update(self):

        waypoints_with_status = self.waypoint_manager.get_all_waypoints_with_status()

        self.waypoint_status_publisher.publish(String(data=str(waypoints_with_status)))

        aruco_finished_mission = self.get_aruco_mission_status()

        self.node.get_logger().info(
            f"Aruco: {aruco_finished_mission} | Finished_mission {self.waypoint_finished_mission}"
        )

        if self.waypoint_finished_mission or aruco_finished_mission:
            self.node.get_logger().info(f"WP {self.waypoint_index}: >>> SUCCESS!!!")
            self.waypoint_manager.update_waypoint_status(self.waypoint_index, True)
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return Status.SUCCESS

        zed_is_object_detected = self.sensor_data.zed_is_object_detected
        zed_is_near_front_obs = self.sensor_data.zed_is_near_front_obs
        lidar_is_near_front_wall = self.sensor_data.lidar_is_near_front_wall
        zed_slope = self.sensor_data.zed_slope

        if zed_is_near_front_obs or lidar_is_near_front_wall:
            return Status.FAILURE
        if zed_is_object_detected and zed_slope == "Flat":
            return Status.FAILURE

        zed_is_near_left_obs = self.sensor_data.zed_is_near_left_obs
        zed_is_near_right_obs = self.sensor_data.zed_is_near_right_obs
        lidar_is_near_left_wall = self.sensor_data.lidar_is_near_left_wall
        lidar_is_near_right_wall = self.sensor_data.lidar_is_near_right_wall

        if (
            zed_is_near_left_obs
            or zed_is_near_right_obs
            or lidar_is_near_left_wall
            or lidar_is_near_right_wall
        ):

            linear_vel = LINEAR_VEL_DICT[zed_slope]

            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            self.node.get_logger().info(f"Near Left or Right Wall")
            return Status.FAILURE

        self.angle_requested_go_right_publisher.publish(Bool(data=False))
        self.angle_requested_go_left_publisher.publish(Bool(data=False))

        x_pos = self.sensor_data.zed_x_pos
        y_pos = self.sensor_data.zed_y_pos
        z_pos = self.sensor_data.zed_z_pos

        x_ori = self.sensor_data.zed_x_ori
        y_ori = self.sensor_data.zed_y_ori
        z_ori = self.sensor_data.zed_z_ori
        w_ori = self.sensor_data.zed_w_ori

        # self.node.get_logger().info(
        #     f"X {round(x_pos,2)} Y {round(y_pos,2)} Z {round(z_pos,2)} | Xo {round(x_ori,2)} Yo {round(y_ori,2)} | Zo {round(z_ori,2)} | Wo {round(w_ori,2)} ||| X {round(self.zed_x_pos,2)} Y {round(self.zed_y_pos,2)} Z {round(self.zed_z_pos,2)} | Xo {round(self.zed_x_ori,2)} Yo {round(self.zed_y_ori,2)} | Zo {round(self.zed_y_ori,2)} | Wo {round(self.zed_w_ori)}"
        # )

        rover_action, waypoint_finished_mission, distance_to_target, diff_angle = (
            self.action_policy.get_action_policy(
                x_pos,
                y_pos,
                z_pos,
                x_ori,
                y_ori,
                z_ori,
                w_ori,
            )
        )
        self.waypoint_finished_mission = waypoint_finished_mission

        angular_vel = ROVER_ACTION_DICT[rover_action]

        # self.feedback_message = f"Action: {rover_action} | Curr WP {self.current_waypoint} | Dist Thresh: {distance_to_target} | Diff Angle {diff_angle}"
        self.feedback_message = f"Current {zed_slope} | Prev: {self.previous_zed_slope}"
        if self.previous_zed_slope == "Incline" and zed_slope == "Flat":
            time.sleep(INCLINE_WAIT)
        if self.previous_zed_slope == "Flat" and zed_slope == "Decline":
            time.sleep(DECLINE_WAIT)
        self.previous_zed_slope = zed_slope

        linear_vel = LINEAR_VEL_DICT[zed_slope]

        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel

        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.distance_to_target_publisher.publish(Float32(data=distance_to_target))
        self.current_waypoint_publisher.publish(Int32(data=self.waypoint_index + 1))
        self.current_waypoint_status_publisher.publish(
            Bool(data=self.waypoint_finished_mission)
        )

        return Status.RUNNING

    def get_aruco_mission_status(self):
        if (
            self.sensor_data.aruco_detected_distance > 0.0
            and self.sensor_data.aruco_detected_distance
            <= ARUCO_FINISHED_MISISON_THRESH
        ) and (self.waypoint_index + 1 == self.sensor_data.aruco_detected_id):
            return True
        return False


class GuardWaypointChecker(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint_manager, waypoint_index, sensor_data):
        super(GuardWaypointChecker, self).__init__(name)
        self.waypoint_index = waypoint_index
        self.waypoint_manager = waypoint_manager

        self.sensor_data = sensor_data

        self.current_waypoint = self.waypoint_manager.get_waypoint(waypoint_index)
        self.action_policy = ActionPolicy(self.current_waypoint)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.waypoint_status_publisher = self.node.create_publisher(
            String, "waypoint_status", TOPIC_QUEUE_SIZE
        )
        self.cmd_vel_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()

    def update(self):
        waypoints_with_status = self.waypoint_manager.get_all_waypoints_with_status()

        self.waypoint_status_publisher.publish(String(data=str(waypoints_with_status)))

        self.node.get_logger().info(f"Current Waypoint {self.current_waypoint}")
        self.node.get_logger().info(f"Current Waypoint Status: {waypoints_with_status}")

        x_pos = self.sensor_data.zed_x_pos
        y_pos = self.sensor_data.zed_y_pos
        z_pos = self.sensor_data.zed_z_pos

        x_ori = self.sensor_data.zed_x_ori
        y_ori = self.sensor_data.zed_y_ori
        z_ori = self.sensor_data.zed_z_ori
        w_ori = self.sensor_data.zed_w_ori

        _, waypoint_finished_mission, _, _ = self.action_policy.get_action_policy(
            x_pos,
            y_pos,
            z_pos,
            x_ori,
            y_ori,
            z_ori,
            w_ori,
        )
        aruco_finished_mission = self.get_aruco_mission_status()

        if waypoint_finished_mission or aruco_finished_mission:
            self.node.get_logger().info(
                f"Waypoint {self.waypoint_index}: >>> SUCCESS!!!"
            )
            self.waypoint_manager.update_waypoint_status(self.waypoint_index, True)

        is_waypoint_visited = self.waypoint_manager.is_waypoint_visited(
            self.waypoint_index
        )

        if is_waypoint_visited:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return Status.SUCCESS
        return Status.FAILURE

    def get_aruco_mission_status(self):
        if (
            self.sensor_data.aruco_detected_distance > 0.0
            and self.sensor_data.aruco_detected_distance
            <= ARUCO_FINISHED_MISISON_THRESH
        ) and (self.waypoint_index + 1 == self.sensor_data.aruco_detected_id):
            return True
        return False


class CheckSideWall(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_data):
        super(CheckSideWall, self).__init__(name)
        self.sensor_data = sensor_data

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.angle_requested_go_right_publisher = self.node.create_publisher(
            Bool, "angle_requested_go_right", 10
        )
        self.angle_requested_go_left_publisher = self.node.create_publisher(
            Bool, "angle_requested_go_left", 10
        )

        self.cmd_vel_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()

    def update(self):
        zed_is_near_front_obs = self.sensor_data.zed_is_near_front_obs
        lidar_is_near_front_wall = self.sensor_data.lidar_is_near_front_wall
        zed_slope = self.sensor_data.zed_slope
        zed_is_object_detected = self.sensor_data.zed_is_object_detected

        if zed_is_near_front_obs or lidar_is_near_front_wall:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            self.node.get_logger().info(f"CheckSideWall Zed Obs | Lidar Front")
            return Status.FAILURE
        if zed_is_object_detected and zed_slope == "Flat":
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            self.node.get_logger().info(f"CheckSideWall Flat")

            return Status.FAILURE

        self.cmd_vel_msg.linear.x = LINEAR_VEL_DICT[zed_slope]

        zed_is_near_left_obs = self.sensor_data.zed_is_near_left_obs
        zed_is_near_right_obs = self.sensor_data.zed_is_near_right_obs
        lidar_is_near_left_wall = self.sensor_data.lidar_is_near_left_wall
        lidar_is_near_right_wall = self.sensor_data.lidar_is_near_right_wall

        self.node.get_logger().info(f"Near Left Wall Zed: {zed_is_near_left_obs}")
        self.node.get_logger().info(f"Near Right Wall Zed: {zed_is_near_right_obs}")

        self.node.get_logger().info(f"Near Left Wall Lidar: {lidar_is_near_left_wall}")
        self.node.get_logger().info(
            f"Near Right Wall Lidar: {lidar_is_near_right_wall}"
        )

        if zed_is_near_left_obs or lidar_is_near_left_wall:
            self.node.get_logger().info("NEAR LEFT!!!!!!!!")
            self.angle_requested_go_right_publisher.publish(Bool(data=True))
            self.cmd_vel_msg.angular.z = -FLICK_ANGLE
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return Status.RUNNING

        if zed_is_near_right_obs or lidar_is_near_right_wall:
            self.node.get_logger().info("NEAR RIGHT!!!!!!!!")
            self.angle_requested_go_left_publisher.publish(Bool(data=True))
            self.cmd_vel_msg.angular.z = FLICK_ANGLE
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return Status.RUNNING

        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class CheckWall(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_data):
        super(CheckWall, self).__init__(name)
        self.sensor_data = sensor_data

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e
        # self.cmd_vel_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        # self.cmd_vel_msg = Twist()

    def update(self):

        zed_is_near_front_obs = self.sensor_data.zed_is_near_front_obs
        lidar_is_near_front_wall = self.sensor_data.lidar_is_near_front_wall

        zed_is_object_detected = self.sensor_data.zed_is_object_detected
        zed_slope = self.sensor_data.zed_slope

        # if zed_is_object_detected and zed_slope == "Flat":
        #     self.cmd_vel_msg.linear.x = 0.0
        #     self.cmd_vel_msg.angular.z = 0.0
        #     self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        # return Status.FAILURE
        if (
            not zed_is_near_front_obs
            and not lidar_is_near_front_wall
            and not self.is_aruco_near()
        ):

            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )

    def is_aruco_near(self):
        if (
            self.sensor_data.aruco_detected_distance > 0.0
            and self.sensor_data.aruco_detected_distance <= ARUCO_NEAR_THRESH
        ) and (self.waypoint_index + 1 == self.sensor_data.aruco_detected_id):
            return True
        return False


class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name, stop_flick=False, sensor_data=None):
        super(Stop, self).__init__(name)
        # self.blackboard = py_trees.blackboard.Blackboard()
        self.stop_flick = stop_flick

        self.sensor_data = sensor_data

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.cmd_vel_msg = Twist()

    def update(self):
        self.node.get_logger().info(f"Update {self.__class__.__name__}")
        if self.sensor_data is not None:
            zed_slope = self.sensor_data.zed_slope
            if zed_slope != "Flat":
                return Status.SUCCESS

        self.node.get_logger().info(
            f"Flick Out: {self.stop_flick} | {self.__class__.__name__}"
        )
        if self.stop_flick:
            self.node.get_logger().info(
                f"Flick {self.stop_flick} | {self.__class__.__name__}"
            )
            self.cmd_vel_msg.linear.x = REVERSE_FLICK_FOR_STOP
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

        time.sleep(FLICK_STOP)

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)

        time.sleep(STOP)
        return Status.SUCCESS

    def terminate(self, new_status):
        time.sleep(STOP)
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class ModePublisher(py_trees.behaviour.Behaviour):
    def __init__(self, name, mode):
        super(ModePublisher, self).__init__(name)
        self.mode = mode

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.mode_publisher = self.node.create_publisher(String, "mode", 10)

        self.mode_msg = String()

    def update(self):
        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        self.mode_msg.data = self.mode
        self.mode_publisher.publish(self.mode_msg)
        time.sleep(STOP)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class RotateToWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint_manager, waypoint_index, sensor_data):
        super(RotateToWaypoint, self).__init__(name)
        self.waypoint_index = waypoint_index
        self.waypoint_manager = waypoint_manager

        self.sensor_data = sensor_data

        self.current_waypoint = self.waypoint_manager.get_waypoint(waypoint_index)
        self.action_policy = ActionPolicy(self.current_waypoint)
        self.waypoint_finished_mission = False

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_msg = Twist()

        self.cmd_vel_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.distance_to_target_publisher = self.node.create_publisher(
            Float32, "/distance_to_target", 10
        )
        self.current_waypoint_publisher = self.node.create_publisher(
            Int32, "/current_waypoint", 10
        )
        self.current_waypoint_status_publisher = self.node.create_publisher(
            Bool, "/current_waypoint_status", 10
        )

    def update(self):

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        x_pos = self.sensor_data.zed_x_pos
        y_pos = self.sensor_data.zed_y_pos
        z_pos = self.sensor_data.zed_z_pos

        x_ori = self.sensor_data.zed_x_ori
        y_ori = self.sensor_data.zed_y_ori
        z_ori = self.sensor_data.zed_z_ori
        w_ori = self.sensor_data.zed_w_ori

        rover_action, finished_mission, distance_to_target, diff_angle = (
            self.action_policy.get_action_policy(
                x_pos,
                y_pos,
                z_pos,
                x_ori,
                y_ori,
                z_ori,
                w_ori,
            )
        )
        self.feedback_message = f"Action: {rover_action}"

        linear_vel = ROVER_ROTATION_DICT[rover_action]

        angular_vel = 0.0

        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel

        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.distance_to_target_publisher.publish(Float32(data=distance_to_target))
        self.current_waypoint_publisher.publish(Int32(data=self.waypoint_index + 1))
        self.current_waypoint_status_publisher.publish(
            Bool(data=self.waypoint_finished_mission)
        )

        self.node.get_logger().info(
            f"Rotate Action: {rover_action} Finished Mission?: {finished_mission}"
        )

        if rover_action == "No Steer":
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


def create_core_waypoint_subtree(waypoint_index, waypoint_manager, sensor_data):

    check_if_waypoint_visited = py_trees.composites.Selector(
        name=f"[{waypoint_index + 1}] WP Status Check", memory=True
    )

    waypoint_sequence = py_trees.composites.Sequence(
        name=f"[{waypoint_index + 1}] Go to WP", memory=True
    )

    stop_and_spin = py_trees.composites.Sequence(
        name=f"[{waypoint_index + 1}] Stop and Spin", memory=True
    )

    stop_action_for_rotate = Stop(
        name=f"[{waypoint_index + 1}] Stop Spin", stop_flick=True
    )

    set_spin_mode = ModePublisher(
        name=f"[{waypoint_index + 1}] Set Spin Mode", mode="spin"
    )
    rotate_to_waypoint = RotateToWaypoint(
        name=f"[{waypoint_index + 1}] Rotate to WP",
        waypoint_manager=waypoint_manager,
        waypoint_index=waypoint_index,
        sensor_data=sensor_data,
    )
    set_mirrored_mode = ModePublisher(
        name=f"[{waypoint_index + 1}] Set Mirrored Mode", mode="mirrored"
    )
    stop_action_for_mirrored = Stop(name=f"[{waypoint_index + 1}] Stop Mirrored")

    object_detection = py_trees.composites.Selector(
        name=f"[{waypoint_index + 1}] Obj Det", memory=True
    )
    check_detected_object = CheckDetectedObject(
        name=f"[{waypoint_index + 1}] Check Obj Det", sensor_data=sensor_data
    )
    stop_action_for_object_detection = Stop(
        name=f"[{waypoint_index + 1}] Stop Action for Obj Det",
        stop_flick=True,
        sensor_data=sensor_data,
    )

    wall_detection = py_trees.composites.Selector(
        name=f"[{waypoint_index + 1}] Wall Detection", memory=True
    )
    guard_check_if_near_wall = CheckWall(
        name=f"[{waypoint_index + 1}] Check Wall", sensor_data=sensor_data
    )

    check_side_wall = CheckSideWall(
        name=f"[{waypoint_index + 1}] Check Side Wall",
        sensor_data=sensor_data,
    )

    go_to_waypoint = GoToWayPoint(
        name=f"[{waypoint_index + 1}] Move to WP",
        waypoint_manager=waypoint_manager,
        waypoint_index=waypoint_index,
        sensor_data=sensor_data,
    )

    guard_check_if_waypoint_visited = GuardWaypointChecker(
        name=f"[{waypoint_index + 1}] Guard - WP Stat Check",
        waypoint_manager=waypoint_manager,
        waypoint_index=waypoint_index,
        sensor_data=sensor_data,
    )

    stop_and_spin.add_children(
        [
            stop_action_for_rotate,
            set_spin_mode,
            rotate_to_waypoint,
            set_mirrored_mode,
            stop_action_for_mirrored,
        ]
    )
    object_detection.add_children(
        [check_detected_object, stop_action_for_object_detection]
    )
    wall_detection.add_children([guard_check_if_near_wall, stop_and_spin])
    waypoint_sequence.add_children(
        [object_detection, wall_detection, check_side_wall, go_to_waypoint]
    )
    check_if_waypoint_visited.add_children(
        [guard_check_if_waypoint_visited, waypoint_sequence]
    )

    return check_if_waypoint_visited


def create_root() -> py_trees.behaviour.Behaviour:

    sensor_data = SensorData()
    waypoint_manager = WaypointManager(waypoints=BASE_WAYPOINT_OBJECTS)

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    sensor_subscribers = SensorSubscribers(
        name="Sensor Subscribers", sensor_data=sensor_data
    )
    go_to_waypoints = py_trees.composites.Sequence(
        name=f"Go to {len(BASE_WAYPOINT_OBJECTS)} Waypoints", memory=True
    )

    for index, _ in enumerate(BASE_WAYPOINT_OBJECTS, start=0):
        go_to_waypoints.add_child(
            create_core_waypoint_subtree(index, waypoint_manager, sensor_data)
        )

    root.add_children([sensor_subscribers, go_to_waypoints])

    return root


def main():

    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=TREE_DEBUG)
    try:
        tree.setup(node_name="GearsTree", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red
            + "failed to setup the tree, aborting [{}]".format(str(e))
            + console.reset
        )
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:

        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=PERIOD_MS)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
