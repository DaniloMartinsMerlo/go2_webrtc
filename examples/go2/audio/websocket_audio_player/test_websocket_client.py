import asyncio
import json
import base64
import sys
import websockets


async def send_audio_file(
    websocket_url: str, audio_file_path: str, audio_format: str = "wav"
):
    """
    Send an audio file to the WebSocket server as base64

    Args:
        websocket_url: WebSocket server URL (e.g., ws://localhost:8765)
        audio_file_path: Path to the audio file to send
        audio_format: Audio format (wav or mp3)
    """
    try:
        # Read audio file
        with open(audio_file_path, "rb") as f:
            audio_bytes = f.read()

        # Encode to base64
        audio_b64 = base64.b64encode(audio_bytes).decode("utf-8")

        # Create message
        message = {"audio": audio_b64, "format": audio_format}

        print(f"Connecting to {websocket_url}...")
        async with websockets.connect(websocket_url) as websocket:
            print(f"Connected! Sending audio file: {audio_file_path}")
            print(f"Audio size: {len(audio_bytes)} bytes")

            # Send message
            await websocket.send(json.dumps(message))
            print("Audio sent, waiting for response...")

            # Wait for responses
            async for response in websocket:
                response_data = json.loads(response)
                status = response_data.get("status")
                message_text = response_data.get("message")

                print(f"[{status.upper()}] {message_text}")

                if status == "success" or status == "error":
                    break

            print("Done!")

    except FileNotFoundError:
        print(f"Error: Audio file not found: {audio_file_path}")
    except Exception as e:
        print(f"Error: {e}")


async def main():
    if len(sys.argv) < 3:
        print(
            "Usage: python test_websocket_client.py <websocket_url> <audio_file_path> [format]"
        )
        print(
            "Example: python test_websocket_client.py ws://192.168.123.100:8765 test.wav wav"
        )
        sys.exit(1)

    websocket_url = sys.argv[1]
    audio_file_path = sys.argv[2]
    audio_format = sys.argv[3] if len(sys.argv) > 3 else "wav"

    await send_audio_file(websocket_url, audio_file_path, audio_format)


if __name__ == "__main__":
    asyncio.run(main())
