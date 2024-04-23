import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import operator
import numpy as np
import time
from py_trees_ros.subscribers import ToBlackboard
from std_msgs.msg import Bool, String
from py_trees.common import Status
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


BASE_WAYPOINT_OBJECTS = [[12.5, 0.0], [12.5, 2.87], [-0.13, 2.87], [-0.13, 29.01]]


class ModePublisher(py_trees.behaviour.Behaviour):
    def __init__(self, name, mode):
        super(ModePublisher, self).__init__(name)
        # self.blackboard = py_trees.blackboard.Blackboard()
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

        # self.node.get_logger().info(f"Mode: {self.mode_msg.data}")
        # time.sleep(1)
        return py_trees.common.Status.SUCCESS

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

        # self.mode_publisher_ = self.node.create_publisher(String, "mode", 10)

        # self.mode_msg = String()

    def initialise(self):
        self.node.get_logger().info(f"Init {self.__class__.__name__}")

    def update(self):
        self.visited_waypoints = self.blackboard.get("visited_waypoints")

        self.node.get_logger().info(f"Update {self.__class__.__name__}")

        is_visited = self.visited_waypoints["wp_" + str(self.wp_ind + 1)]

        self.node.get_logger().info(
            f"Update, Is WP {self.wp_ind} Visited? {is_visited}"
        )

        # self.visited_waypoints["wp_" + str(self.wp_inf + 1)]
        if is_visited:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"Terminate {self.__class__.__name__} - {new_status}"
        )


class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Stop, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

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

        return py_trees.common.Status.SUCCESS

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
        self.cmd_vel_msg = Twist()

    def initialise(self):

        self.node.get_logger().info(f"Init {self.__class__.__name__}")
        self.visited_waypoints = self.blackboard.get("visited_waypoints")

    def update(self):
        is_obj_detected = self.blackboard.get("object_detected")
        is_near_wall = self.blackboard.get("near_wall")

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        x_pos = self.blackboard.get("pos_x")
        y_pos = self.blackboard.get("pos_y")
        z_pos = self.blackboard.get("pos_z")
        x_ori = self.blackboard.get("ori_x")
        y_ori = self.blackboard.get("ori_y")
        z_ori = self.blackboard.get("ori_z")
        w_ori = self.blackboard.get("ori_w")

        rover_action, finished_mission = self.action_policy.get_action_policy(
            x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori
        )

        self.node.get_logger().info(
            f"Go To WP Act {self.wp_ind + 1}: {rover_action} Finished Mission?: {finished_mission}"
        )

        if rover_action == "Left Steer":
            angular_vel = 3.0
        elif rover_action == "Right Steer":
            angular_vel = -3.0
        else:
            angular_vel = 0.0

        linear_vel = 3.0

        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel

        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)

        if finished_mission:
            self.visited_waypoints["wp_" + str(self.wp_ind + 1)] = True
            self.blackboard.set("visited_waypoints", self.visited_waypoints)
            return py_trees.common.Status.SUCCESS

        if is_obj_detected or is_near_wall:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
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

    def initialise(self):

        self.node.get_logger().info(f"Init {self.__class__.__name__}")
        time.sleep(1)

    def update(self):
        is_obj_detected = self.blackboard.get("object_detected")
        # is_near_wall = self.blackboard.get("near_wall")

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        x_pos = self.blackboard.get("pos_x")
        y_pos = self.blackboard.get("pos_y")
        z_pos = self.blackboard.get("pos_z")
        x_ori = self.blackboard.get("ori_x")
        y_ori = self.blackboard.get("ori_y")
        z_ori = self.blackboard.get("ori_z")
        w_ori = self.blackboard.get("ori_w")

        rover_action, finished_mission = self.action_policy.get_action_policy(
            x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori
        )

        if rover_action == "Left Steer":
            linear_vel = 1.0
        elif rover_action == "Right Steer":
            linear_vel = -1.0
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
            return py_trees.common.Status.SUCCESS

        if is_obj_detected:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(self.cmd_vel_msg)
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

    is_object_detected = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Object Detected? {waypoint_index}",
        check=py_trees.common.ComparisonExpression(
            variable="object_detected", value=False, operator=operator.eq
        ),
    )

    is_near_wall = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Near Wall? {waypoint_index}",
        check=py_trees.common.ComparisonExpression(
            variable="near_wall", value=False, operator=operator.eq
        ),
    )

    # obj_det_inv = py_trees.decorators.Inverter(
    #     name=f"Obj Inv {waypoint_index}", child=is_object_detected
    # )

    # near_wall_inv = py_trees.decorators.Inverter(
    #     name=f"Near Wall Inv {waypoint_index}", child=is_near_wall
    # )

    stop_action_for_obj_det = Stop(name=f"Stop Obj{waypoint_index}")
    stop_action_for_rotate = Stop(name=f"Stop Spin{waypoint_index}")
    stop_action_for_mirrored = Stop(name=f"Stop mirrored {waypoint_index}")

    mov_to_wp = GoToWayPoint(
        name=f"Move to WP #{waypoint_index}",
        waypoint=waypoint[waypoint_index - 1],
        wp_ind=waypoint_index,
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

    wp_sequence.add_child(obj_detection)
    wp_sequence.add_child(wall_detection)
    wp_sequence.add_child(mov_to_wp)
    check_wp_before_moving.add_children([guard_wp_checker, wp_sequence])

    return check_wp_before_moving


def create_root() -> py_trees.behaviour.Behaviour:
    blackboard = py_trees.blackboard.Blackboard()

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # visited_waypoints = {
    #     "wp_1": False,
    #     "wp_2": False,
    #     "wp_3": False,
    #     "wp_4": False,
    # }
    visited_waypoints = {
        "wp_" + str(i + 1): False for i in range(len(BASE_WAYPOINT_OBJECTS))
    }

    blackboard.set("visited_waypoints", visited_waypoints)

    go_to_waypoints = py_trees.composites.Sequence(name="Go to Waypoints", memory=True)

    topics_to_bb = py_trees.composites.Sequence(
        name="Topics To Blackboard", memory=True
    )

    obj_to_blackboard = ToBlackboard(
        name="object to bb",
        topic_name="is_object_detected",
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"object_detected": "data"},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )

    near_wall_to_blackboard = ToBlackboard(
        name="near wall to bb",
        topic_name="is_near_wall",
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"near_wall": "data"},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )

    odom_bb_var = {
        "pos_x": "pose.pose.position.x",
        "pos_y": "pose.pose.position.y",
        "pos_z": "pose.pose.position.z",
        "ori_x": "pose.pose.orientation.x",
        "ori_y": "pose.pose.orientation.y",
        "ori_z": "pose.pose.orientation.z",
        "ori_w": "pose.pose.orientation.w",
    }
    odom_to_blackboard = ToBlackboard(
        name="odom to bb",
        topic_name="odom",
        topic_type=Odometry,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables=odom_bb_var,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )

    # mode_to_blackboard = ToBlackboard(
    #     name="mode to bb",
    #     topic_name="mode",
    #     topic_type=String,
    #     qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    #     blackboard_variables={"mode": None},
    #     clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    # )

    root.add_child(topics_to_bb)
    root.add_child(go_to_waypoints)

    topics_to_bb.add_children(
        [
            obj_to_blackboard,
            odom_to_blackboard,
            near_wall_to_blackboard,
            # mode_to_blackboard,
        ]
    )

    for index, _ in enumerate(BASE_WAYPOINT_OBJECTS, start=1):
        go_to_waypoints.add_child(create_core_wp_subtree(index, BASE_WAYPOINT_OBJECTS))

    return root


def main():

    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
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

    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
