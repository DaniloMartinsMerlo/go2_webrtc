# go2_commander/main.py
import os
import argparse
import threading
import uvicorn
import rclpy

from commander_core import CommanderCore
from http_api import create_app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoints", type=str, default=None, help="Path to waypoints JSON")
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    # precedence: CLI arg > ENV > default path
    wp_file = args.waypoints or os.getenv("GO2_WAYPOINTS") or "waypoints/waypoints.json"

    rclpy.init()
    core = CommanderCore()

    # load waypoints automatically (throws if missing/invalid)
    try:
        core.load_waypoints(wp_file)
    except Exception as e:
        core.get_logger().error(f"Failed to load waypoints '{wp_file}': {e}")
        # shutdown ROS properly
        core.destroy_node()
        rclpy.shutdown()
        return

    app = create_app(core)

    # Run Uvicorn (FastAPI) in a separate daemon thread
    def run_uvicorn():
        uvicorn.run(app, host=args.host, port=args.port)

    api_thread = threading.Thread(target=run_uvicorn, daemon=True)
    api_thread.start()
    core.get_logger().info(f"HTTP API running on http://{args.host}:{args.port} (Swagger: /docs)")

    # Keep ROS spinning in main thread (timers and nav ticks execute here)
    try:
        rclpy.spin(core)
    except KeyboardInterrupt:
        core.get_logger().info("KeyboardInterrupt, shutting down")
    finally:
        core.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
