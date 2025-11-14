import aiohttp
import asyncio
import argparse


BASE_URL = "http://localhost:8080"


async def send_request(action: str):
    url = f"{BASE_URL}/{action}"
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(url) as response:
                text = await response.text()
                print(f"✅ [{action.upper()}] Resposta do servidor: {text}")
        except aiohttp.ClientError as e:
            print(f"❌ Erro ao enviar requisição: {e}")


async def main():
    parser = argparse.ArgumentParser(description="Cliente para controle do robô")
    parser.add_argument(
        "action",
        choices=["play", "stop"],
        help="Ação a ser enviada ao robô"
    )
    args = parser.parse_args()
    await send_request(args.action)


if __name__ == "__main__":
    asyncio.run(main())
