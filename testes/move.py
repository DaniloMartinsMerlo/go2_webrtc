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

        # print("Robot connected, checking current motion mode...")
        # response = await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["MOTION_SWITCHER"], 
        #     {"api_id": 1001}
        # )

        # if response['data']['header']['status']['code'] == 0:
        #     data = json.loads(response['data']['data'])
        #     current_motion_switcher_mode = data['name']
        #     print(f"Current motion mode: {current_motion_switcher_mode}")

        # if current_motion_switcher_mode != "normal":
        #     print(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")
        #     await conn.datachannel.pub_sub.publish_request_new(
        #         RTC_TOPIC["MOTION_SWITCHER"], 
        #         {
        #             "api_id": 1002,
        #             "parameter": {"name": "normal"}
        #         }
        #     )
        #     await asyncio.sleep(10)

        movement_duration = 2
        start_time = time.time()
        end_time = start_time + movement_duration

        print(f"Starting forward movement for {movement_duration} seconds")

        avoid_set = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["OBSTACLES_AVOID"], 
            {
                "api_id": 1001,
                "parameter": {"enable": True}
            }
        )
        print(avoid_set)
        
        while time.time() <= end_time:

            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 1, "y": 0, "z": 0}
                }
            )
            
            await asyncio.sleep(0.95)
            
            avoid_read = await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["OBSTACLES_AVOID"], 
                {
                    "api_id": 1002,
                    # "parameter": {"enable": True}
                }
            )

            print(avoid_read)
            
        print("Done.")

    except ValueError as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)