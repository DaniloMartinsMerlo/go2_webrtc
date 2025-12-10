import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph


class Go2Commander(Node):
    def __init__(self):
        super().__init__("go2_commander")
        self.waypoints_data = {}

        # 1. Setup Map Loader Client
        self.map_loader = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map"
        )

    def load_map(self, map_name):
        """Loads a map serialized by SLAM Toolbox"""
        self.get_logger().info(f"Attempting to load map: {map_name}...")

        while not self.map_loader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "SLAM Toolbox deserialize service not available, waiting..."
            )

        req = DeserializePoseGraph.Request()
        req.filename = map_name
        req.match_type = 1  # START_AT_GIVEN_POSE = 1, START_AT_FIRST_NODE = 0

        # We assume the map loads exactly where we saved it
        # If match_type = 1, we define the initial pose here (optional)
        req.initial_pose.x = 0.0
        req.initial_pose.y = 0.0
        req.initial_pose.theta = 0.0

        future = self.map_loader.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Map loaded successfully!")
        else:
            self.get_logger().error("Failed to load map.")

    def load_waypoints(self, filename="waypoints.json"):
        with open(filename, "r") as f:
            self.waypoints_data = json.load(f)

        self.checkpoint_names = list(self.waypoints_data.keys())
        self.get_logger().info(f"Loaded checkpoints: {self.checkpoint_names}")

    def pose_from_checkpoint(self, checkpoint_name, navigator):
        data = self.waypoints_data[checkpoint_name]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()

        pose.pose.position.x = data["position_x"]
        pose.pose.position.y = data["position_y"]
        pose.pose.position.z = data.get("position_z", 0.0)

        pose.pose.orientation.x = data["orientation_x"]
        pose.pose.orientation.y = data["orientation_y"]
        pose.pose.orientation.z = data["orientation_z"]
        pose.pose.orientation.w = data["orientation_w"]

        return pose


def main():
    rclpy.init()

    # Initialize our helper node for loading maps
    commander_node = Go2Commander()

    # Initialize the Nav2 Commander
    nav = BasicNavigator()

    # --- TASK 1: LOAD MAP ---
    # NOTE: You must have saved a map previously using the Serialize service!
    # Example filename: /home/unitree/my_map (without extension)
    # map_file = "/home/unitree/go2_ros2_toolbox/lab"
    # commander_node.load_map(map_file)

    # Wait for Nav2 to be fully active
    # We assume the robot is already localized because we loaded the SLAM graph
    nav.waitUntilNav2Active(localizer="bt_navigator")

    # --- LOAD WAYPOINTS ---
    commander_node.load_waypoints("waypoints.json")

    # --- EXECUTE ALL CHECKPOINTS ---
    for cp_name in commander_node.checkpoint_names:
        print(f"\n➡️ Navigating to checkpoint: {cp_name}")

        goal_pose = commander_node.pose_from_checkpoint(cp_name, nav)
        nav.goToPose(goal_pose)

        # i = 0
        # initial_distance = None

        while not nav.isTaskComplete():
            # i += 1
            # feedback = nav.getFeedback()

            # if feedback:
            #     dist_remaining = feedback.distance_remaining

            #     if initial_distance is None:
            #         initial_distance = dist_remaining

            #     if i % 20 == 0:
            #         percent = 100 * (1 - dist_remaining / initial_distance)
            #         print(f"{cp_name} -> {dist_remaining:.2f}m ({percent:.1f}%)")

            rclpy.spin_once(commander_node, timeout_sec=0.1)

        result = nav.getResult()
        print(f"Checkpoint {cp_name} result: {result}")

    # --- RESULT HANDLING ---
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
