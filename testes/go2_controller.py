import asyncio
import json
import time
import math
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class Go2Controller:    
    
    async def __init__(self, conn, ip):
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip)
        await conn.connect()        

    async def connect(self, ip):
        self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip)
        await self.conn.connect()

    async def normal(self):
        print("Robot connected, checking current motion mode...")
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], 
            {"api_id": 1001}
        )

        if response['data']['header']['status']['code'] == 0:
            data = json.loads(response['data']['data'])
            current_motion_switcher_mode = data['name']
            print(f"Current motion mode: {current_motion_switcher_mode}")

            if current_motion_switcher_mode != "normal":

                print(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")

                await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["MOTION_SWITCHER"], 
                    {
                        "api_id": 1002,
                        "parameter": {"name": "normal"}
                    }
                )
                await asyncio.sleep(5)

    async def move(self, x_speed, x_distance, y_speed, y_distance):

        if (x_speed & y_speed):
            print("Invalid movement")
            return

        if (x_speed != 0):
            x_speed =  constrain(x_speed, -2.5, 3.8)
            movement_duration = (x_distance/x_speed)
        
        if (y_speed != 0):
            y_speed = constrain(y_speed, -1, 1)
            movement_duration = (y_distance/y_speed) 
        
        start_time = time.time()
        end_time = start_time + movement_duration
        
        print(f"Starting forward movement for {movement_duration} seconds")

        while time.time() <= end_time:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": x_speed, "y": y_speed, "z": 0}
                }
            )
            await asyncio.sleep(0.9)

        print("Done.")
    

    async def turn(self, z_speed, z_degrees):

        z_speed = constrain(z_speed, -4, 4)
        radians = math.radians(z_degrees)
        movement_duration = (radians/z_speed)
        
        start_time = time.time()
        end_time = start_time + movement_duration

        print(f"Starting turn movement for {movement_duration} seconds")

        while time.time() <= end_time:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], 
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 0, "y": 0, "z": z_speed}
                }
            )
            await asyncio.sleep(0.9)

        print("Done.")
    
    async def stop(self):
        
        print("Stopping robot...")    
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StopMove"]
            }
        )
        await asyncio.sleep(1)
    
    async def hard_stop(self):
        print("Stopping robot...")    
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["Damp"]
            }
        )
        await asyncio.sleep(1)