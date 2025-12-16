import logging
import asyncio
import json
import base64
import tempfile
import os
import websockets
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebSocketAudioPlayer:
    def __init__(self, robot_ip: str, ws_host: str = "0.0.0.0", ws_port: int = 8765):
        """
        Initialize WebSocket Audio Player

        Args:
            robot_ip: IP address of the Unitree robot
            ws_host: WebSocket server host (default: 0.0.0.0)
            ws_port: WebSocket server port (default: 8765)
        """
        self.robot_ip = robot_ip
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.webrtc_conn = None
        self.audio_hub = None
        self.temp_dir = tempfile.mkdtemp()
        logger.info(f"Temporary directory created: {self.temp_dir}")

    async def initialize_robot_connection(self):
        """Establish WebRTC connection with the robot"""
        try:
            logger.info(f"Connecting to robot at {self.robot_ip}")
            self.webrtc_conn = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA, ip=self.robot_ip
            )
            await self.webrtc_conn.connect()
            logger.info("WebRTC connection established")

            # Initialize audio hub
            self.audio_hub = WebRTCAudioHub(self.webrtc_conn, logger)
            logger.info("Audio hub initialized")
        except Exception as e:
            logger.error(f"Failed to connect to robot: {e}")
            raise

    async def play_base64_audio(self, audio_data_b64: str, audio_format: str = "wav"):
        """
        Decode base64 audio and play it on the robot

        Args:
            audio_data_b64: Base64 encoded audio data
            audio_format: Audio format (wav or mp3, default: wav)
        """
        try:
            # Decode base64 audio
            logger.info("Decoding base64 audio data")
            audio_bytes = base64.b64decode(audio_data_b64)

            # Create temporary file
            temp_filename = f"audio_{asyncio.get_event_loop().time()}.{audio_format}"
            temp_filepath = os.path.join(self.temp_dir, temp_filename)

            # Write audio to temporary file
            with open(temp_filepath, "wb") as f:
                f.write(audio_bytes)
            logger.info(f"Audio saved to temporary file: {temp_filepath}")

            # Upload and play audio
            logger.info("Uploading audio to robot...")
            await self.audio_hub.upload_audio_file(temp_filepath)
            logger.info("Audio uploaded successfully")

            # Get the UUID of the uploaded file
            response = await self.audio_hub.get_audio_list()
            if response and isinstance(response, dict):
                data_str = response.get("data", {}).get("data", "{}")
                audio_list = json.loads(data_str).get("audio_list", [])

                # Get the filename without extension
                filename = os.path.splitext(temp_filename)[0]

                # Find the uploaded audio
                existing_audio = next(
                    (audio for audio in audio_list if audio["CUSTOM_NAME"] == filename),
                    None,
                )

                if existing_audio:
                    uuid = existing_audio["UNIQUE_ID"]
                    logger.info(f"Playing audio with UUID: {uuid}")
                    await self.audio_hub.play_by_uuid(uuid)
                    logger.info("Audio playback started")
                else:
                    logger.error("Could not find uploaded audio in list")

            # Cleanup temporary file
            try:
                os.remove(temp_filepath)
                logger.info(f"Temporary file removed: {temp_filepath}")
            except Exception as e:
                logger.warning(f"Failed to remove temporary file: {e}")

        except Exception as e:
            logger.error(f"Error playing audio: {e}")
            raise

    async def handle_websocket_connection(self, websocket):
        """Handle incoming WebSocket connections"""
        client_addr = websocket.remote_address
        logger.info(f"New WebSocket connection from {client_addr}")

        try:
            async for message in websocket:
                try:
                    # Parse JSON message
                    data = json.loads(message)

                    # Extract audio data and format
                    audio_b64 = data.get("audio")
                    audio_format = data.get("format", "wav")

                    if not audio_b64:
                        error_msg = "Missing 'audio' field in message"
                        logger.error(error_msg)
                        await websocket.send(
                            json.dumps({"status": "error", "message": error_msg})
                        )
                        continue

                    logger.info(f"Received audio data (format: {audio_format})")

                    # Send acknowledgment
                    await websocket.send(
                        json.dumps(
                            {
                                "status": "received",
                                "message": "Audio data received, processing...",
                            }
                        )
                    )

                    # Play the audio
                    await self.play_base64_audio(audio_b64, audio_format)

                    # Send success response
                    await websocket.send(
                        json.dumps(
                            {
                                "status": "success",
                                "message": "Audio played successfully",
                            }
                        )
                    )

                except json.JSONDecodeError as e:
                    error_msg = f"Invalid JSON: {e}"
                    logger.error(error_msg)
                    await websocket.send(
                        json.dumps({"status": "error", "message": error_msg})
                    )
                except Exception as e:
                    error_msg = f"Error processing audio: {e}"
                    logger.error(error_msg)
                    await websocket.send(
                        json.dumps({"status": "error", "message": error_msg})
                    )

        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            logger.info(f"WebSocket connection closed: {client_addr}")

    async def start_server(self):
        """Start the WebSocket server"""
        logger.info(f"Starting WebSocket server on {self.ws_host}:{self.ws_port}")

        async with websockets.serve(
            self.handle_websocket_connection, self.ws_host, self.ws_port
        ):
            logger.info(
                f"WebSocket server running on ws://{self.ws_host}:{self.ws_port}"
            )
            logger.info("Waiting for connections...")
            logger.info(
                'Expected message format: {"audio": "<base64_encoded_audio>", "format": "wav"}'
            )
            await asyncio.Future()  # Run forever

    async def run(self):
        """Main entry point"""
        try:
            # Initialize robot connection
            await self.initialize_robot_connection()

            # Start WebSocket server
            await self.start_server()

        except KeyboardInterrupt:
            logger.info("Server stopped by user")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            # Cleanup
            try:
                import shutil

                shutil.rmtree(self.temp_dir)
                logger.info(f"Cleaned up temporary directory: {self.temp_dir}")
            except Exception as e:
                logger.warning(f"Failed to clean up temporary directory: {e}")


async def main():
    # Configuration
    ROBOT_IP = "192.168.123.161"  # Change this to your robot's IP
    WS_HOST = "0.0.0.0"  # Listen on all interfaces
    WS_PORT = 8765  # WebSocket port

    # Create and run the player
    player = WebSocketAudioPlayer(ROBOT_IP, WS_HOST, WS_PORT)
    await player.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\nProgram interrupted by user")
