import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import operator
import numpy as np
import time
from py_trees_ros.subscribers import ToBlackboard
from std_msgs.msg import Bool, String, Float32
from py_trees.common import Status
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_srvs.srv import Trigger

# BASE_WAYPOINT_OBJECTS = [[2.7, 0.0], [2.2, 3.4]]  # OFFICE TEST 2 PTS
# BASE_WAYPOINT_OBJECTS = [[12.5, 0.0], [12.5, 2.87], [-0.13, 2.87], [-0.13, 29.01]]
BASE_WAYPOINT_OBJECTS = [
    [3.33, 0.20],
    [2.1, 2.3],
    [1.84, -0.34],
]
# OFFICE COMPLETE SQUARE
# BASE_WAYPOINT_OBJECTS = [[10.6, 0], [10.6, 2.2], [-0.5, 2.2], [-0.5, 44]] #RAMP PHYSICS

LINEAR_VEL = 3.0
ANGULAR_VEL = 3.0
ROTATION_VEL = 1.5

MAX_ROT_CORRECTION = 0.075
FLICK_ANGLE = 0.17

ZED_ODOM_TOPIC = "/odom"

PERIOD_MS = 125  # MS
TREE_DEBUG = True
# LINEAR_VEL = 0.0
# ANGULAR_VEL = 0.0


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

        self.mode_publisher_ = self.node.create_publisher(String, "mode", 10)

        self.mode_msg = String()

    def update(self):
        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        self.mode_msg.data = self.mode
        self.mode_publisher_.publish(self.mode_msg)
        time.sleep(1)

        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class CheckVisitedWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint_ind):
        super(CheckVisitedWaypoint, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.wp_ind = waypoint_ind - 1

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

    def initialise(self):
        self.node.get_logger().info(f"Init {self.__class__.__name__}")

    def update(self):
        self.visited_waypoints = self.blackboard.get("visited_waypoints")

        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        is_visited = self.visited_waypoints["wp_" + str(self.wp_ind + 1)]

        self.node.get_logger().info(
            f"Update, Is WP {self.wp_ind} Visited? {is_visited}"
        )

        if is_visited:
            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Stop, self).__init__(name)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_publisher_ = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.cmd_vel_msg = Twist()

    def update(self):
        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)

        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class CheckWall(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckWall, self).__init__(name)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.wall_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_wall", self.wall_collision_callback, 10
        )

        self.is_near_wall = False

    def wall_collision_callback(self, msg):
        self.is_near_wall = msg.data

    def update(self):

        if not self.is_near_wall:

            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class CheckSideWall(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckSideWall, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.wall_left_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_left_wall", self.wall_left_collision_callback, 10
        )

        self.wall_right_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_right_wall", self.wall_right_collision_callback, 10
        )

        # self.curr_angle_subscriber = self.node.create_subscription(
        #     Float32, "/current_angle", self.current_angle_callback, 10
        # )

        self.angle_requested_go_right_publisher_ = self.node.create_publisher(
            Bool, "angle_requested_go_right", 10
        )
        self.angle_requested_go_left_publisher_ = self.node.create_publisher(
            Bool, "angle_requested_go_left", 10
        )

        self.is_near_left_wall = False
        self.is_near_right_wall = False
        self.curr_angle = 0.0

        self.cmd_vel_publisher_ = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()

    def wall_left_collision_callback(self, msg):
        self.is_near_left_wall = msg.data

    def wall_right_collision_callback(self, msg):
        self.is_near_right_wall = msg.data

    def current_angle_callback(self, msg):
        self.curr_angle = msg.data

    def update(self):

        self.node.get_logger().info(f"Near Left Wall: {self.is_near_left_wall}")
        self.node.get_logger().info(f"Near Right Wall: {self.is_near_right_wall}")

        self.cmd_vel_msg.linear.x = LINEAR_VEL

        if self.is_near_left_wall:
            self.angle_requested_go_right_publisher_.publish(Bool(data=True))
            self.cmd_vel_msg.angular.z = -FLICK_ANGLE
            self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
            return Status.RUNNING

        if self.is_near_right_wall:
            self.angle_requested_go_left_publisher_.publish(Bool(data=True))
            self.cmd_vel_msg.angular.z = FLICK_ANGLE
            self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
            return Status.RUNNING

        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class CheckObj(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObj, self).__init__(name)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.obj_detection_subscriber = self.node.create_subscription(
            Bool, "/is_object_detected", self.obj_detected_callback, 10
        )

        self.is_obj_detected = False

    def obj_detected_callback(self, msg):
        self.is_obj_detected = msg.data

    def update(self):

        if not self.is_obj_detected:

            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class ActionPolicy:
    def __init__(self, waypoint):
        self.waypoint = waypoint
        self.DIFF_ANGLE_THRESHOLD = 3  # DEGREE
        self.DISTANCE_TO_TARGET_THRESHOLD = (
            0.5  # METERSDIFF_ANGLE_THRESHOLD = 1  # DEGREE
        )
        self.finished_mission = False

    def get_action_policy(self, x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori):

        yaw = np.degrees(
            np.arctan2(
                2 * (x_ori * y_ori + w_ori * z_ori),
                1 - 2 * (y_ori * y_ori + z_ori * z_ori),
            )
        )

        target_position = self.waypoint

        relDisX = target_position[0] - x_pos
        relDisY = target_position[1] - y_pos

        if relDisX > 0 and relDisY > 0:
            theta = np.arctan(relDisY / relDisX)
        elif relDisX > 0 and relDisY < 0:
            theta = 2 * np.pi + np.arctan(relDisY / relDisX)
        elif relDisX < 0 and relDisY < 0:
            theta = np.pi + np.arctan(relDisY / relDisX)
        elif relDisX < 0 and relDisY > 0:
            theta = np.pi + np.arctan(relDisY / relDisX)
        elif relDisX == 0 and relDisY > 0:
            theta = 1 / 2 * np.pi
        elif relDisX == 0 and relDisY < 0:
            theta = 3 / 2 * np.pi
        elif relDisY == 0 and relDisX > 0:
            theta = 0
        else:
            theta = np.pi

        relTheta = np.degrees(theta)
        diffAngle = relTheta - yaw

        if diffAngle <= 180:
            diffAngle = diffAngle
        else:
            diffAngle = diffAngle - 360.0

        if abs(diffAngle) > self.DIFF_ANGLE_THRESHOLD:
            # if diffAngle >= DIFF_ANGLE_THRESHOLD:
            #     rover_action = "Left Steer"  # LEFT_STEER
            # elif diffAngle <= -DIFF_ANGLE_THRESHOLD:
            #     rover_action = "Right Steer"  # RIGHT_STEER
            if diffAngle > 0:
                rover_action = "Left Steer"  # LEFT_STEER
            # elif diffAngle <= -DIFF_ANGLE_THRESHOLD:
            else:
                rover_action = "Right Steer"

        else:
            rover_action = "No Steer"  # NO_STEER

        distance_to_target = np.sqrt(
            (x_pos - target_position[0]) ** 2 + (y_pos - target_position[1]) ** 2
        )

        if distance_to_target < self.DISTANCE_TO_TARGET_THRESHOLD:
            self.finished_mission = True

        print(f"{distance_to_target} - {rover_action}")

        return rover_action, self.finished_mission


class GoToWayPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint, wp_ind):
        super(GoToWayPoint, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

        # self.waypoint = waypoint
        self.name = name
        self.wp_ind = wp_ind - 1
        self.action_policy = ActionPolicy(waypoint)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_publisher_ = self.node.create_publisher(Twist, "cmd_vel", 10)

        # self.angle_requested_publisher_ = self.node.create_publisher(
        #     Bool, "angle_requested", 10
        # )

        self.zed_odom_subscriber = self.node.create_subscription(
            Odometry,
            ZED_ODOM_TOPIC,  # Change 'odometry_topic' to the actual topic name
            self.odometry_callback,
            10,  # Adjust queue size as per your requirement
        )

        self.wall_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_wall", self.wall_collision_callback, 10
        )

        self.obj_detection_subscriber = self.node.create_subscription(
            Bool, "/is_object_detected", self.obj_detected_callback, 10
        )

        (
            self.x_pos,
            self.y_pos,
            self.z_pos,
            self.x_ori,
            self.y_ori,
            self.z_ori,
            self.w_ori,
        ) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.is_obj_detected, self.is_near_wall = (False, False)

        self.cmd_vel_msg = Twist()

        self.wall_left_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_left_wall", self.wall_left_collision_callback, 10
        )

        self.wall_right_collision_subscriber = self.node.create_subscription(
            Bool, "/is_near_right_wall", self.wall_right_collision_callback, 10
        )

        self.angle_requested_go_right_publisher_ = self.node.create_publisher(
            Bool, "angle_requested_go_right", 10
        )
        self.angle_requested_go_left_publisher_ = self.node.create_publisher(
            Bool, "angle_requested_go_left", 10
        )
        self.is_near_left_wall = False
        self.is_near_right_wall = False

    def wall_left_collision_callback(self, msg):
        self.is_near_left_wall = msg.data

    def wall_right_collision_callback(self, msg):
        self.is_near_right_wall = msg.data

    def wall_collision_callback(self, msg):
        self.is_near_wall = msg.data

    def obj_detected_callback(self, msg):
        self.is_obj_detected = msg.data
        # self.get_logger().info('I heard: "%s"' % msg.data)

    def odometry_callback(self, msg):
        # Extracting position
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = msg.pose.pose.position.z

        # Extracting orientation (rotation)
        self.x_ori = msg.pose.pose.orientation.x
        self.y_ori = msg.pose.pose.orientation.y
        self.z_ori = msg.pose.pose.orientation.z
        self.w_ori = msg.pose.pose.orientation.w

    def initialise(self):

        self.node.get_logger().info(f"Init {self.__class__.__name__}")
        self.visited_waypoints = self.blackboard.get("visited_waypoints")

    def update(self):

        if self.is_obj_detected or self.is_near_wall:
            return Status.FAILURE

        if self.is_near_left_wall or self.is_near_right_wall:
            self.node.get_logger().info(f"Near Left or Right Wall")

            return Status.FAILURE

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        self.angle_requested_go_right_publisher_.publish(Bool(data=False))
        self.angle_requested_go_left_publisher_.publish(Bool(data=False))

        rover_action, finished_mission = self.action_policy.get_action_policy(
            self.x_pos,
            self.y_pos,
            self.z_pos,
            self.x_ori,
            self.y_ori,
            self.z_ori,
            self.w_ori,
        )

        self.node.get_logger().info(
            f"Go To WP Act {self.wp_ind + 1}: {rover_action} Finished Mission?: {finished_mission}"
        )

        if rover_action == "Left Steer":
            angular_vel = ANGULAR_VEL
        elif rover_action == "Right Steer":
            angular_vel = -ANGULAR_VEL
        else:
            angular_vel = 0.0

        linear_vel = LINEAR_VEL

        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel

        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)

        if finished_mission:
            self.visited_waypoints["wp_" + str(self.wp_ind + 1)] = True
            self.blackboard.set("visited_waypoints", self.visited_waypoints)
            return Status.SUCCESS

        return Status.RUNNING
        # except Exception as e:
        #     self.node.get_logger().info(f"Error: {e}")

    def terminate(self, new_status):

        # self.cmd_vel_msg.linear.x = 0.0
        # self.cmd_vel_msg.angular.z = 0.0
        # self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class RotateToWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint):
        super(RotateToWaypoint, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

        # self.waypoint = waypoint
        self.name = name
        self.action_policy = ActionPolicy(waypoint)

    def setup(self, **kwargs):

        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e

        self.node.get_logger().info(f"Setup {self.__class__.__name__}")

        self.cmd_vel_publisher_ = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()
        self.temp_action = None

        self.zed_odom_subscriber = self.node.create_subscription(
            Odometry,
            ZED_ODOM_TOPIC,  # Change 'odometry_topic' to the actual topic name
            self.odometry_callback,
            10,  # Adjust queue size as per your requirement
        )

        (
            self.x_pos,
            self.y_pos,
            self.z_pos,
            self.x_ori,
            self.y_ori,
            self.z_ori,
            self.w_ori,
        ) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.obj_detection_subscriber = self.node.create_subscription(
            Bool, "/is_object_detected", self.obj_detected_callback, 10
        )
        self.is_obj_detected = False

        self.cmd_vel_msg = Twist()

    def obj_detected_callback(self, msg):
        self.is_obj_detected = msg.data

    def odometry_callback(self, msg):
        # Extracting position
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = msg.pose.pose.position.z

        # Extracting orientation (rotation)
        self.x_ori = msg.pose.pose.orientation.x
        self.y_ori = msg.pose.pose.orientation.y
        self.z_ori = msg.pose.pose.orientation.z
        self.w_ori = msg.pose.pose.orientation.w

    def initialise(self):

        self.node.get_logger().info(f"Init {self.__class__.__name__}")
        time.sleep(1)

    def update(self):
        # try:
        # is_obj_detected = self.blackboard.get("object_detected")
        # is_obj_detected = False

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        rover_action, finished_mission = self.action_policy.get_action_policy(
            self.x_pos,
            self.y_pos,
            self.z_pos,
            self.x_ori,
            self.y_ori,
            self.z_ori,
            self.w_ori,
        )

        if rover_action == "Left Steer":
            linear_vel = ROTATION_VEL
        elif rover_action == "Right Steer":
            linear_vel = -ROTATION_VEL
        else:
            linear_vel = 0.0

        angular_vel = 0.0

        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel

        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
        self.node.get_logger().info(
            f"Rotate Action: {rover_action} Finished Mission?: {finished_mission}"
        )
        if rover_action == "No Steer":
            return Status.SUCCESS

        if self.is_obj_detected:
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):

        # self.cmd_vel_msg.linear.x = 0.0
        # self.cmd_vel_msg.angular.z = 0.0
        # self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


def create_core_wp_subtree(waypoint_index, waypoint):

    guard_wp_checker = CheckVisitedWaypoint(
        name=f"Guard WP Check {waypoint_index}", waypoint_ind=waypoint_index
    )

    check_wp_before_moving = py_trees.composites.Selector(
        name=f"Check Visited WP {waypoint_index}", memory=True
    )

    wp_sequence = py_trees.composites.Sequence(
        name=f"Go to WP #{waypoint_index}", memory=True
    )

    is_object_detected = CheckObj(name=f"Check Obj Det {waypoint_index}")

    is_near_wall = CheckWall(name=f"Check Wall {waypoint_index}")

    stop_action_for_obj_det = Stop(name=f"Stop Obj{waypoint_index}")
    stop_action_for_rotate = Stop(name=f"Stop Spin{waypoint_index}")
    stop_action_for_mirrored = Stop(name=f"Stop mirrored {waypoint_index}")

    mov_to_wp = GoToWayPoint(
        name=f"Move to WP #{waypoint_index}",
        waypoint=waypoint[waypoint_index - 1],
        wp_ind=waypoint_index,
    )

    check_side_wall = CheckSideWall(
        name=f"Check Side Wall #{waypoint_index}",
    )

    rot_to_wp = RotateToWaypoint(
        name=f"Rotate to WP #{waypoint_index}",
        waypoint=waypoint[waypoint_index - 1],
    )

    set_spin_mode = ModePublisher(name="Set Spin Mode", mode="spin")
    set_mirrored_mode = ModePublisher(name="Set mirrored Mode", mode="mirrored")

    obj_detection = py_trees.composites.Selector(
        name=f"Obj Detection {waypoint_index}", memory=True
    )
    obj_detection.add_children([is_object_detected, stop_action_for_obj_det])
    # obj_detection.add_child(is_object_detected)

    wall_detection = py_trees.composites.Selector(
        name=f"Wall Detection {waypoint_index}", memory=True
    )

    stop_and_spin = py_trees.composites.Sequence(
        name=f"Stop and Spin {waypoint_index}", memory=True
    )
    wall_detection.add_children([is_near_wall, stop_and_spin])
    stop_and_spin.add_children(
        [
            stop_action_for_rotate,
            set_spin_mode,
            rot_to_wp,
            set_mirrored_mode,
            stop_action_for_mirrored,
        ]
    )
    # stop_and_spin.add_children([stop_action_for_rotate, set_spin_mode])

    wp_sequence.add_children(
        [obj_detection, wall_detection, check_side_wall, mov_to_wp]
    )

    check_wp_before_moving.add_children([guard_wp_checker, wp_sequence])

    return check_wp_before_moving


def create_root() -> py_trees.behaviour.Behaviour:
    blackboard = py_trees.blackboard.Blackboard()

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    visited_waypoints = {
        "wp_" + str(i + 1): False for i in range(len(BASE_WAYPOINT_OBJECTS))
    }

    blackboard.set("visited_waypoints", visited_waypoints)
    blackboard.set("corrected", False)

    go_to_waypoints = py_trees.composites.Sequence(name="Go to Waypoints", memory=True)

    root.add_child(go_to_waypoints)

    for index, _ in enumerate(BASE_WAYPOINT_OBJECTS, start=1):
        go_to_waypoints.add_child(create_core_wp_subtree(index, BASE_WAYPOINT_OBJECTS))

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
