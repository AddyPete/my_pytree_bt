import py_trees
from py_trees.common import Status
import py_trees_ros.trees
import py_trees.console as console
from py_trees_ros.subscribers import ToBlackboard
from std_msgs.msg import Bool, String
import rclpy
import sys
from geometry_msgs.msg import Twist
import operator
from nav_msgs.msg import Odometry
import numpy as np

BASE_WAYPOINT_OBJECTS = [[12.5, 0.0], [12.5, 2.87], [-0.13, 2.87], [-0.13, 29.01]]


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


class GoToWayPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, waypoint):
        super(GoToWayPoint, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.DIFF_ANGLE_THRESHOLD = 5  # DEGREE
        self.DISTANCE_TO_TARGET_THRESHOLD = (
            0.2  # METERSDIFF_ANGLE_THRESHOLD = 1  # DEGREE
        )
        self.waypoint = waypoint
        self.name = name

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

        self.finished_mission = False

        self.node.get_logger().info(f"Init {self.__class__.__name__}")

    def update(self):
        is_obj_detected = self.blackboard.get("object_detected")

        self.node.get_logger().info(f"Update  {self.__class__.__name__} - {self.name}")

        x_pos = self.blackboard.get("pos_x")
        y_pos = self.blackboard.get("pos_y")
        z_pos = self.blackboard.get("pos_z")
        x_ori = self.blackboard.get("ori_x")
        y_ori = self.blackboard.get("ori_y")
        z_ori = self.blackboard.get("ori_z")
        w_ori = self.blackboard.get("ori_w")

        rover_action, finished_mission = self.action_policy(
            x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori
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

    def action_policy(self, x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori):

        yaw = np.degrees(
            np.arctan2(
                2 * (x_ori * y_ori + w_ori * z_ori),
                1 - 2 * (y_ori * y_ori + z_ori * z_ori),
            )
        )

        # target_position = self.waypoint_objects[self.current_waypoint]
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

        if diffAngle >= self.DIFF_ANGLE_THRESHOLD:
            rover_action = "Left Steer"  # LEFT_STEER
        elif diffAngle < -self.DIFF_ANGLE_THRESHOLD:
            rover_action = "Right Steer"  # RIGHT_STEER

        else:
            rover_action = "No Steer"  # NO_STEER

        distance_to_target = np.sqrt(
            (x_pos - target_position[0]) ** 2 + (y_pos - target_position[1]) ** 2
        )

        if distance_to_target < self.DISTANCE_TO_TARGET_THRESHOLD:
            self.finished_mission = True

        return rover_action, self.finished_mission


def create_detector_sequence(waypoint_index, waypoint_objects):
    """
    Creates a sequence for going to a waypoint and stopping if an object is detected.
    """
    sequence = py_trees.composites.Sequence(
        name=f"Go to WP #{waypoint_index}", memory=True
    )

    is_object_detected = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Object Detected?",
        check=py_trees.common.ComparisonExpression(
            variable="object_detected", value=True, operator=operator.eq
        ),
    )

    obj_det_inv = py_trees.decorators.Inverter(name="Obj Inv", child=is_object_detected)
    stop_action = Stop(name="Stop")
    mov_to_wp = GoToWayPoint(
        name=f"Move to WP #{waypoint_index}",
        waypoint=waypoint_objects[waypoint_index - 1],
    )

    obj_detection = py_trees.composites.Selector(name="Obj Detection", memory=True)
    obj_detection.add_children([obj_det_inv, stop_action])

    sequence.add_child(obj_detection)
    sequence.add_child(mov_to_wp)

    return sequence


def create_root() -> py_trees.behaviour.Behaviour:

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    go_to_waypoints = py_trees.composites.Sequence(name="Go to WPS", memory=True)

    obj_det_1 = py_trees.composites.Selector(name="Obj Detection", memory=True)
    obj_det_2 = py_trees.composites.Selector(name="Obj Detection", memory=True)

    go_to_wp_1 = py_trees.composites.Sequence(name="Go to WP #1", memory=True)
    go_to_wp_2 = py_trees.composites.Sequence(name="Go to WP #2", memory=True)

    topics_to_bb = py_trees.composites.Sequence(name="topics_to_bb", memory=True)

    obj_to_blackboard = ToBlackboard(
        name="object to bb",
        topic_name="is_object_detected",
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"object_detected": "data"},
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

    is_object_detected_1 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Object Detected?",
        check=py_trees.common.ComparisonExpression(
            variable="object_detected", value=True, operator=operator.eq
        ),
    )
    is_object_detected_2 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Object Detected?",
        check=py_trees.common.ComparisonExpression(
            variable="object_detected", value=True, operator=operator.eq
        ),
    )

    stop_action_1 = Stop(name="Stop")
    stop_action_2 = Stop(name="Stop")

    mov_to_wp_1 = GoToWayPoint(name="Move to WP #1", waypoint=BASE_WAYPOINT_OBJECTS[0])
    mov_to_wp_2 = GoToWayPoint(name="Move to WP #2", waypoint=BASE_WAYPOINT_OBJECTS[1])

    obj_det_inv_1 = py_trees.decorators.Inverter(
        name="Obj Inv", child=is_object_detected_1
    )
    obj_det_inv_2 = py_trees.decorators.Inverter(
        name="Obj Inv", child=is_object_detected_2
    )

    root.add_child(topics_to_bb)
    root.add_child(go_to_waypoints)
    go_to_waypoints.add_child(go_to_wp_1)
    go_to_waypoints.add_child(go_to_wp_2)

    topics_to_bb.add_child(obj_to_blackboard)
    topics_to_bb.add_child(odom_to_blackboard)

    go_to_wp_1.add_child(obj_det_1)
    go_to_wp_1.add_child(mov_to_wp_1)
    obj_det_1.add_children([obj_det_inv_1, stop_action_1])

    go_to_wp_2.add_child(obj_det_2)
    go_to_wp_2.add_child(mov_to_wp_2)
    obj_det_2.add_children([obj_det_inv_2, stop_action_2])
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
