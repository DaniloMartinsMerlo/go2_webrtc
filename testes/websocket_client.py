"""
Cliente WebSocket simples para testar o servidor do robô Go2

Uso:
    python websocket_client.py listen       # Monitora eventos em tempo real
    python websocket_client.py play         # Executa próximo checkpoint
    python websocket_client.py stop         # Para o robô
    python websocket_client.py status       # Consulta status
"""
import aiohttp
import asyncio
import argparse
import json
from datetime import datetime


WS_URL = "ws://localhost:8080/ws"


def log_event(event_data: dict):
    """Exibe evento recebido"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    event = event_data.get("event", "unknown")
    
    print(f"\n[{timestamp}] 📡 {event.upper()}")
    
    # Exibe campos relevantes
    for key, value in event_data.items():
        if key != "event":
            print(f"  {key}: {value}")


async def websocket_listener():
    """Conecta ao WebSocket e escuta eventos do robô"""
    print(f"🔌 Conectando ao WebSocket: {WS_URL}")
    
    async with aiohttp.ClientSession() as session:
        try:
            async with session.ws_connect(WS_URL) as ws:
                print("✅ Conectado ao servidor do robô!")
                print("📊 Aguardando eventos...\n")
                
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        try:
                            data = json.loads(msg.data)
                            log_event(data)
                        except json.JSONDecodeError:
                            print(f"❌ Erro ao decodificar JSON: {msg.data}")
                    
                    elif msg.type == aiohttp.WSMsgType.ERROR:
                        print(f"❌ Erro no WebSocket: {ws.exception()}")
                        break
                    
                    elif msg.type == aiohttp.WSMsgType.CLOSED:
                        print("🔌 Conexão WebSocket fechada")
                        break
        
        except aiohttp.ClientError as e:
            print(f"❌ Erro ao conectar: {e}")
        except KeyboardInterrupt:
            print("\n👋 Desconectando...")


async def send_websocket_command(action: str):
    """Envia um comando via WebSocket"""
    print(f"📤 Enviando comando: {action}")
    
    async with aiohttp.ClientSession() as session:
        try:
            async with session.ws_connect(WS_URL) as ws:
                # Envia comando
                await ws.send_json({"action": action})
                print(f"✅ Comando '{action}' enviado!")
                
                # Aguarda e exibe resposta
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        data = json.loads(msg.data)
                        
                        # Ignora mensagem inicial de conexão
                        if data.get('event') == 'connected':
                            continue
                        
                        log_event(data)
                        
                        # Se recebeu resposta do comando, pode sair
                        if data.get('event') in [f"{action}_response", "error", "status"]:
                            break
                
                await ws.close()
                
        except aiohttp.ClientError as e:
            print(f"❌ Erro ao enviar comando: {e}")


async def main():
    parser = argparse.ArgumentParser(
        description="Cliente WebSocket para testar o robô Go2"
    )
    
    parser.add_argument(
        "action",
        choices=["listen", "play", "stop", "status"],
        help="Ação a ser executada"
    )
    
    args = parser.parse_args()
    
    if args.action == "listen":
        await websocket_listener()
    else:
        command = "get_status" if args.action == "status" else args.action
        await send_websocket_command(command)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Programa interrompido pelo usuário.")

