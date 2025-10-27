import asyncio
import logging
import sys
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)
    
async def main():
    try:
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.123.161")
        await conn.connect()

        print("Moving forward...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["Move"],
                "parameter": {"x": 1, "y": 0, "z": 0}
            }
        )

        await asyncio.sleep(3)

        # # Perform a "Move Backward" movement
        # print("Moving backward...")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["Move"],
        #         "parameter": {"x": -0.5, "y": 0, "z": 0}
        #     }
        # )

        await asyncio.sleep(3600)
    
    except ValueError as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)