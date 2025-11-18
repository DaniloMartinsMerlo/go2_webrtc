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

        movement_duration = 1.5
        start_time = time.time()
        foward_time = start_time + movement_duration

        print(f"\n Starting forward movement for {movement_duration} seconds \n")
        
        while time.time() <= foward_time:
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 1, "y": 0, "z": 0}
                }
            )
            await asyncio.sleep(0.95)

        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StopMove"]
            }
        )

        time.sleep(3)

        print("Turning \n")        

        movement_duration = 3
        start_time = time.time()
        turn_time = start_time + movement_duration

        while time.time() <= turn_time:

            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 0, "y": 0, "z": -1}
                }
            )
            
            await asyncio.sleep(0.95)
        
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StopMove"]
            }
        )

        movement_duration = 1.5
        start_time = time.time()
        foward_time = start_time + movement_duration

        while time.time() <= foward_time:

            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 1, "y": 0, "z": 0}
                }
            )
            
            await asyncio.sleep(0.95)
        
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StopMove"]
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