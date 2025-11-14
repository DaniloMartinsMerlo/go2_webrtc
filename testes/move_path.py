import asyncio
import json
import logging
import sys
from aiohttp import web
from go2_controller import Go2Controller
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

logging.basicConfig(level=logging.INFO)
WAYPOINTS_PATH = "waypoints.json"


class RobotController:
    def __init__(self):
        self.command_queue = asyncio.Queue()
        self.movement_done = asyncio.Event()
        self.current_checkpoint_index = 0
        self.checkpoint_names = []
        self.waypoints_data = {}
        self.is_running = False
        self.go2 = None

    async def initialize(self):
        await self.load_waypoints()

        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.189")
        await conn.connect()
        self.go2 = Go2Controller(conn)
        self.go2.set_on_motion_complete(lambda: self.movement_done.set())

        logging.info("🤖 Robô conectado e pronto.")

    async def load_waypoints(self):
        try:
            with open(WAYPOINTS_PATH, "r") as f:
                self.waypoints_data = json.load(f)
                self.checkpoint_names = list(self.waypoints_data.keys())
                logging.info(f"Checkpoints carregados: {self.checkpoint_names}")
        except FileNotFoundError:
            logging.error("Arquivo de waypoints não encontrado.")
            sys.exit(1)

    async def populate_queue_for_checkpoint(self, index):
        if index >= len(self.checkpoint_names):
            logging.info("✅ Todos os checkpoints foram executados.")
            return False

        checkpoint = self.checkpoint_names[index]
        commands = self.waypoints_data[checkpoint]
        logging.info(f"[Checkpoint] Populando fila '{checkpoint}' ({len(commands)} comandos)")
        for cmd in commands:
            await self.command_queue.put(cmd)
        return True

    async def execute_command(self, cmd):
        cmd_type = cmd["cmd"]
        params = cmd.get("params", {})
        logging.info(f"[Executor] '{cmd_type}' com parâmetros {params}")

        match cmd_type:
            case "Move":
                await self.go2.move(**params)
                await self.go2.stop()
            case "Turn":
                await self.go2.turn(**params)
                await self.go2.stop()
            case "Stop":
                await self.go2.stop()
            case "Hard Stop":
                await self.go2.hard_stop()
            case _:
                logging.warning(f"[Executor] Comando desconhecido: {cmd_type}")

    async def consumer(self):
        while not self.command_queue.empty():
            cmd = await self.command_queue.get()
            try:
                await self.execute_command(cmd)
                await self.movement_done.wait()
                self.movement_done.clear()
            finally:
                self.command_queue.task_done()

        logging.info(f"[Checkpoint {self.checkpoint_names[self.current_checkpoint_index]}] finalizado ✅")
        self.is_running = False
        self.current_checkpoint_index += 1

    async def play_next_checkpoint(self):
        if self.is_running:
            logging.info("⏳ Já executando um checkpoint.")
            return {"status": "busy"}

        if self.current_checkpoint_index >= len(self.checkpoint_names):
            logging.info("🏁 Todos os checkpoints foram executados.")
            return {"status": "finished"}

        await self.populate_queue_for_checkpoint(self.current_checkpoint_index)
        self.is_running = True
        asyncio.create_task(self.consumer())
        return {
            "status": "started",
            "checkpoint": self.checkpoint_names[self.current_checkpoint_index]
        }

    def get_status(self):
        return {
            "running": self.is_running,
            "current_checkpoint": (
                self.checkpoint_names[self.current_checkpoint_index]
                if self.current_checkpoint_index < len(self.checkpoint_names)
                else None
            ),
            "remaining": len(self.checkpoint_names) - self.current_checkpoint_index,
        }


async def handle_play(request):
    robot: RobotController = request.app["robot"]
    result = await robot.play_next_checkpoint()
    return web.json_response(result)


async def handle_status(request):
    robot: RobotController = request.app["robot"]
    return web.json_response(robot.get_status())


async def start_web_server(robot: RobotController):
    app = web.Application()
    app["robot"] = robot
    app.router.add_post("/play", handle_play)
    app.router.add_get("/status", handle_status)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    logging.info("🌐 Servidor do robô ouvindo em http://localhost:8080")


async def main():
    robot = RobotController()
    await robot.initialize()
    await start_web_server(robot)

    logging.info("⏸️ Aguardando comandos /play ou /status...")
    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário.")
        sys.exit(0)
