import asyncio
import logging
import sys
import json
from go2_controller import Go2Controller

waypoints_path = "waipoints.json"

Go2 = Go2Controller(ip="192.168.0.189")

try:
    with open(waypoints_path, mode="r") as path:
        waypoints = json.load(path)
except FileNotFoundError:
    print("Waypoints file not found")


logging.basicConfig(level=logging.FATAL)

async def main():
    try:
        for commands in waypoints:
            match commands["cmd"]:
                case "Move":
                    Go2.move(**commands)
                
                case "Turn":
                    Go2.turn(**commands)

                case "Stop":
                    Go2.stop()

                case "Hard Stop":
                    Go2.hard_stop(**commands)
                

    except ValueError as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)