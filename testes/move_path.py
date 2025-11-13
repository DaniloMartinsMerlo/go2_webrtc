import asyncio
import json
import logging
import sys
from go2_controller import Go2Controller
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

WAYPOINTS_PATH = "waypoints.json"
logging.basicConfig(level=logging.INFO)

command_queue = asyncio.Queue()
movement_done = asyncio.Event()

async def load_waypoints():
    try:
        with open(WAYPOINTS_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        logging.error("Waypoints file not found.")
        sys.exit(1)

async def producer(waypoints):
    for cmd in waypoints:
        await command_queue.put(cmd)
        logging.info(f"[Producer] Adicionando comando: {cmd}")
    logging.info("[Producer] Todos os comandos foram adicionados.")

async def consumer(go2: Go2Controller):
    while True:
        cmd = await command_queue.get()
        try:
            await execute_command(go2, cmd)

            logging.info("[Consumer] Aguardando robô terminar movimento...")
            await movement_done.wait()

            movement_done.clear()
            logging.info("[Consumer] Movimento finalizado, pegando próximo.")
        
        except Exception as e:
            logging.error(f"Erro executando comando {cmd}: {e}")
        
        finally:
            command_queue.task_done()

async def execute_command(go2: Go2Controller, cmd):
    cmd_type = cmd["cmd"]
    params = cmd.get("params", {})

    logging.info(f"[Consumer] Executando comando '{cmd_type}' com params {params}")

    match cmd_type:
        case "Move":
            await go2.move(**params)
        case "Turn":
            await go2.turn(**params)
        case "Stop":
            await go2.stop()
        case "Hard Stop":
            await go2.hard_stop()
        case _:
            logging.warning(f"[Consumer] Comando desconhecido: {cmd_type}")

async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.189")
    await conn.connect()
    logging.info("🤖 WebRTC conectado com sucesso")

    go2 = Go2Controller(conn)

    def on_motion_complete():
        logging.info("[Callback] Movimento concluído ✅")
        movement_done.set()

    go2.set_on_motion_complete(on_motion_complete)

    waypoints = await load_waypoints()

    producer_task = asyncio.create_task(producer(waypoints))
    consumer_task = asyncio.create_task(consumer(go2))

    await producer_task
    await command_queue.join()
    consumer_task.cancel()

    logging.info("✅ Todos os comandos foram processados.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)