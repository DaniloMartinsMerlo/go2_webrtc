import asyncio
import logging
import sys
import json
import time
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

logging.basicConfig(level=logging.FATAL)

async def main():
    try:
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.189")
        await conn.connect()

        movement_duration = 2
        start_time = time.time()
        foward_time = start_time + movement_duration

        print(f"\n Starting forward movement for {movement_duration} seconds \n")
        
        while time.time() <= foward_time:
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Damp"]
                }
            )
            await asyncio.sleep(0.95)

        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StandUp"]
            }
        )

        
        print("Done.\n")
        
    except ValueError as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)