# go2_commander/commander_core.py
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from waypoint_manager import WaypointManager


class CommanderCore(Node):
    def __init__(self):
        super().__init__("go2_commander_core")

        self.get_logger().info("Inicializando BasicNavigator (aguardando Nav2)...")
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")
        self.get_logger().info("Nav2 ativo!")

        self.wp = WaypointManager()

        # protected state
        self._lock = threading.RLock()
        self._current_checkpoint = None        # nome do checkpoint alvo/atual
        self._pending_pose = None              # PoseStamped que será enviada no próximo tick
        self._pending_name = None
        self._navigating = False
        self._hold_enabled = False
        self._last_message = ""

        # timer ROS que executa na thread do rclpy.spin()
        self.create_timer(0.2, self._nav_tick)

    # ----------------------------
    # Waypoints
    # ----------------------------
    def load_waypoints(self, filename: str):
        names = self.wp.load(filename)
        with self._lock:
            self._last_message = f"{len(names)} waypoints carregados"
        self.get_logger().info(self._last_message)
        return names

    # ----------------------------
    # Helpers
    # ----------------------------
    def _pose_from_payload(self, payload) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(payload["position_x"])
        pose.pose.position.y = float(payload["position_y"])
        pose.pose.position.z = float(payload.get("position_z", 0.0))

        pose.pose.orientation.x = float(payload.get("orientation_x", 0.0))
        pose.pose.orientation.y = float(payload.get("orientation_y", 0.0))
        pose.pose.orientation.z = float(payload.get("orientation_z", 0.0))
        pose.pose.orientation.w = float(payload["orientation_w"])

        return pose

    # ----------------------------
    # Timer tick (roda na thread ROS)
    # ----------------------------
    def _nav_tick(self):
        # start pending goal if exists and not navigating
        with self._lock:
            if self._pending_pose is not None and not self._navigating:
                pose = self._pending_pose
                name = self._pending_name
                # clear pending before calling goToPose to avoid races
                self._pending_pose = None
                self._pending_name = None
                self._navigating = True
                self._current_checkpoint = name
                self._last_message = f"Dispatched goal -> {name}"
                self.get_logger().info(self._last_message)

                # start navigation (this will internally use rclpy futures)
                self.navigator.goToPose(pose)
                return

        # if navigating, check completion
        with self._lock:
            if self._navigating:
                if self.navigator.isTaskComplete():
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self._last_message = f"Arrived at {self._current_checkpoint}"
                        self.get_logger().info(self._last_message)
                    elif result == TaskResult.CANCELED:
                        self._last_message = f"Navigation canceled ({self._current_checkpoint})"
                        self.get_logger().warn(self._last_message)
                    else:
                        self._last_message = f"Navigation failed ({self._current_checkpoint})"
                        self.get_logger().error(self._last_message)
                    self._navigating = False

    # ----------------------------
    # Public API (thread-safe; called from HTTP thread)
    # ----------------------------
    def start_checkpoint(self, name: str):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"
            if name not in self.wp.names:
                return False, f"Checkpoint '{name}' not found"
            if self._navigating:
                return False, "Already navigating"

            payload = self.wp.get(name)
            pose = self._pose_from_payload(payload)
            self._pending_pose = pose
            self._pending_name = name
            self._last_message = f"Queued {name}"
            return True, f"Queued start for {name}"

    def start_next(self):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"
            if self.wp.count() == 0:
                return False, "No waypoints loaded"
            if self._current_checkpoint in self.wp.names:
                idx = self.wp.names.index(self._current_checkpoint) + 1
            else:
                idx = 0
            if idx >= len(self.wp.names):
                return False, "No next waypoint"
            next_name = self.wp.names[idx]
        return self.start_checkpoint(next_name)

    def pause(self):
        with self._lock:
            if not self._navigating:
                return False, "Not navigating"
            self.navigator.cancelTask()
            self._last_message = "Pause requested (cancelled goal)"
            return True, "Paused (cancelled current goal)"

    def resume(self):
        with self._lock:
            if self._hold_enabled:
                return False, "Hold enabled"
            if self._navigating:
                return False, "Already navigating"
            if not self._current_checkpoint:
                return False, "No checkpoint to resume"
            # re-queue same checkpoint
            name = self._current_checkpoint
        return self.start_checkpoint(name)

    def cancel(self):
        with self._lock:
            if not self._navigating:
                return False, "Not navigating"
            self.navigator.cancelTask()
            self._last_message = "Cancel requested"
            return True, "Cancel requested"

    def set_hold(self, enabled: bool):
        with self._lock:
            self._hold_enabled = bool(enabled)
            return True, f"Hold set to {self._hold_enabled}"

    def status(self) -> dict:
        with self._lock:
            return {
                "current_checkpoint": self._current_checkpoint or "",
                "navigating": self._navigating,
                "hold_enabled": self._hold_enabled,
                "last_message": self._last_message,
                "waypoints_count": self.wp.count(),
            }
