import aiohttp
import asyncio

async def trigger_play():
    url = "http://localhost:8080/play"
    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(url) as response:
                text = await response.text()
                print(f"✅ Resposta do servidor: {text}")
        except aiohttp.ClientError as e:
            print(f"❌ Erro ao enviar requisição: {e}")

if __name__ == "__main__":
    asyncio.run(trigger_play())
