import asyncio
import json
import logging
import sys
from datetime import datetime, timezone
from aiohttp import web
from go2_controller import Go2Controller
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

logging.basicConfig(level=logging.INFO)
WAYPOINTS_PATH = "waypoints.json"


class RobotController:
    def __init__(self):
        self.command_queue = asyncio.Queue()
        self.movement_done = asyncio.Event()
        self.stop_event = asyncio.Event()
        self.current_checkpoint_index = 0
        self.checkpoint_names = []
        self.waypoints_data = {}
        self.is_running = False
        self.go2 = None
        self.websocket_clients = set()  # Para armazenar clientes WebSocket conectados
        self.checkpoint_inicio_real = None  # Timestamp de início do checkpoint atual

    async def load_waypoints(self):
        try:
            with open(WAYPOINTS_PATH, "r") as f:
                self.waypoints_data = json.load(f)
                self.checkpoint_names = list(self.waypoints_data.keys())
                logging.info(f"Checkpoints carregados: {self.checkpoint_names}")
        except FileNotFoundError:
            logging.error("Arquivo de waypoints não encontrado.")
            sys.exit(1)

    async def connect_robot(self):
        """Tenta conectar ao robô físico (não bloqueia o WebSocket)"""
        try:
            logging.info("🔄 Tentando conectar ao robô...")
            conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.189")
            await conn.connect()
            self.go2 = Go2Controller(conn)
            self.go2.set_on_motion_complete(lambda: self.movement_done.set())
            logging.info("✅ Robô conectado e pronto!")
            
            # Notifica clientes WebSocket que o robô conectou
            await self.broadcast_to_websockets({
                "event": "robot_connected",
                "status": "online"
            })
        except (Exception, SystemExit) as e:
            logging.warning(f"⚠️ Falha ao conectar no robô: {e}")
            logging.warning("⚠️ Servidor WebSocket continuará em MODO OFFLINE")
            self.go2 = None

    async def load_waypoints(self):
        try:
            with open(WAYPOINTS_PATH, "r") as f:
                self.waypoints_data = json.load(f)
                self.checkpoint_names = list(self.waypoints_data.keys())
                logging.info(f"✅ Checkpoints carregados: {self.checkpoint_names}")
        except FileNotFoundError:
            logging.error("❌ Arquivo de waypoints não encontrado.")
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
        """Executa um comando, respeitando o stop_event"""
        cmd_type = cmd["cmd"]
        params = cmd.get("params", {})
        logging.info(f"[Executor] '{cmd_type}' com parâmetros {params}")

        if self.stop_event.is_set():
            logging.warning("[Executor] Stop recebido — abortando comando atual.")
            return

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
        """Executa todos os comandos do checkpoint atual"""
        checkpoint_name = self.checkpoint_names[self.current_checkpoint_index]
        self.checkpoint_inicio_real = datetime.now(timezone.utc)
        
        # Notifica início do checkpoint - dados para o backend atualizar a tabela
        await self.broadcast_to_websockets({
            "event": "checkpoint_started",
            "tipo": checkpoint_name,  # Nome do local (recepção, auditório, etc)
            "ordem": self.current_checkpoint_index + 1,  # Ordem no tour (1-based)
            "status": "running",
            "inicio_real": self.checkpoint_inicio_real.isoformat()
        })
        
        while not self.command_queue.empty():
            if self.stop_event.is_set():
                logging.warning("[Consumer] Stop solicitado — interrompendo checkpoint atual.")
                break

            cmd = await self.command_queue.get()
            try:
                await self.execute_command(cmd)
                await self.movement_done.wait()
                self.movement_done.clear()
            finally:
                self.command_queue.task_done()

        if self.stop_event.is_set():
            while not self.command_queue.empty():
                self.command_queue.get_nowait()
                self.command_queue.task_done()

        logging.info(f"[Checkpoint {checkpoint_name}] finalizado ✅")
        
        fim_real = datetime.now(timezone.utc)
        
        # Notifica conclusão do checkpoint - dados para o backend atualizar a tabela
        await self.broadcast_to_websockets({
            "event": "checkpoint_completed",
            "tipo": checkpoint_name,
            "ordem": self.current_checkpoint_index + 1,
            "status": "skipped" if self.stop_event.is_set() else "finished",
            "inicio_real": self.checkpoint_inicio_real.isoformat(),
            "fim_real": fim_real.isoformat()
        })
        
        self.is_running = False
        self.current_checkpoint_index += 1
        self.checkpoint_inicio_real = None

    async def play_next_checkpoint(self):
        # VALIDAÇÃO: Robô deve estar conectado para executar checkpoints
        if self.go2 is None:
            logging.error("❌ Robô não conectado - comando 'play' rejeitado")
            return {
                "status": "error",
                "error": "robot_not_connected",
                "message": "Robô não está conectado. Não é possível executar checkpoints."
            }

        if self.is_running:
            logging.info("⏳ Já executando um checkpoint.")
            return {"status": "busy"}

        if self.current_checkpoint_index >= len(self.checkpoint_names):
            logging.info("🏁 Todos os checkpoints foram executados.")
            return {"status": "finished"}

        self.stop_event.clear()

        await self.populate_queue_for_checkpoint(self.current_checkpoint_index)
        self.is_running = True
        asyncio.create_task(self.consumer())
        return {
            "status": "started",
            "checkpoint": self.checkpoint_names[self.current_checkpoint_index]
        }

    async def emergency_stop(self):
        """Para tudo imediatamente"""
        if self.go2 is None:
            logging.warning("⚠️ Robô não conectado - comando 'stop' ignorado")
            return {"status": "error", "error": "robot_not_connected"}

        logging.warning("🛑 Recebido STOP — interrompendo robô.")
        self.stop_event.set()

        # Notifica parada de emergência
        await self.broadcast_to_websockets({
            "event": "emergency_stop",
            "tipo": self.checkpoint_names[self.current_checkpoint_index] if self.current_checkpoint_index < len(self.checkpoint_names) else None,
            "ordem": self.current_checkpoint_index + 1 if self.current_checkpoint_index < len(self.checkpoint_names) else None
        })

        try:
            await self.go2.stop()
            await self.go2.stand()
            logging.info("✅ Robô parado com segurança (stop + stand).")
        except Exception as e:
            logging.error(f"Erro ao parar o robô: {e}")

        # limpa fila
        while not self.command_queue.empty():
            self.command_queue.get_nowait()
            self.command_queue.task_done()

        self.is_running = False

        return {"status": "stopped"}

    def get_status(self):
        """Retorna o status atual do robô"""
        return {
            "robot_connected": self.go2 is not None,
            "is_running": self.is_running,
            "current_checkpoint": self.checkpoint_names[self.current_checkpoint_index] if self.current_checkpoint_index < len(self.checkpoint_names) else None,
            "current_checkpoint_index": self.current_checkpoint_index,
            "total_checkpoints": len(self.checkpoint_names),
            "checkpoint_names": self.checkpoint_names,
            "queue_size": self.command_queue.qsize(),
            "is_stopped": self.stop_event.is_set()
        }

    async def broadcast_to_websockets(self, message: dict):
        """Envia mensagem para todos os clientes WebSocket conectados"""
        if not self.websocket_clients:
            return
        
        message_str = json.dumps(message)
        disconnected_clients = set()
        
        for ws in self.websocket_clients:
            try:
                await ws.send_str(message_str)
            except Exception as e:
                logging.warning(f"Erro ao enviar para cliente WebSocket: {e}")
                disconnected_clients.add(ws)
        
        # Remove clientes desconectados
        self.websocket_clients -= disconnected_clients


# -------------------------------
# Rotas HTTP
# -------------------------------
async def handle_websocket(request):
    """Handler para conexões WebSocket"""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    robot: RobotController = request.app["robot"]
    robot.websocket_clients.add(ws)
    
    logging.info(f"🔌 Novo cliente WebSocket conectado. Total: {len(robot.websocket_clients)}")
    
    # Envia status inicial
    await ws.send_str(json.dumps({
        "event": "connected",
        "status": robot.get_status()
    }))
    
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    action = data.get("action")
                    
                    if action == "get_status":
                        await ws.send_str(json.dumps({
                            "event": "status",
                            "data": robot.get_status()
                        }))
                    elif action == "play":
                        result = await robot.play_next_checkpoint()
                        await ws.send_str(json.dumps({
                            "event": "play_response",
                            "data": result
                        }))
                    elif action == "stop":
                        result = await robot.emergency_stop()
                        await ws.send_str(json.dumps({
                            "event": "stop_response",
                            "data": result
                        }))
                    else:
                        await ws.send_str(json.dumps({
                            "event": "error",
                            "message": f"Ação desconhecida: {action}"
                        }))
                        
                except json.JSONDecodeError:
                    await ws.send_str(json.dumps({
                        "event": "error",
                        "message": "Mensagem inválida (esperado JSON)"
                    }))
                    
            elif msg.type == web.WSMsgType.ERROR:
                logging.error(f'WebSocket connection closed with exception {ws.exception()}')
                
    finally:
        robot.websocket_clients.discard(ws)
        logging.info(f"🔌 Cliente WebSocket desconectado. Total: {len(robot.websocket_clients)}")
    
    return ws


async def handle_play(request):
    robot: RobotController = request.app["robot"]
    result = await robot.play_next_checkpoint()
    return web.json_response(result)


async def handle_status(request):
    robot: RobotController = request.app["robot"]
    return web.json_response(robot.get_status())


async def handle_stop(request):
    robot: RobotController = request.app["robot"]
    result = await robot.emergency_stop()
    return web.json_response(result)


# -------------------------------
# Servidor interno do robô
# -------------------------------
async def start_web_server(robot: RobotController):
    app = web.Application()
    app["robot"] = robot
    
    # Rotas HTTP
    app.router.add_post("/play", handle_play)
    app.router.add_post("/stop", handle_stop)
    app.router.add_get("/status", handle_status)
    
    # Rota WebSocket
    app.router.add_get("/ws", handle_websocket)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    logging.info("🌐 Servidor do robô ouvindo em http://localhost:8080")
    logging.info("🔌 WebSocket disponível em ws://localhost:8080/ws")


# -------------------------------
# MAIN
# -------------------------------
async def main():
    robot = RobotController()
    await robot.load_waypoints()  # Carrega waypoints primeiro
    
    # Inicia WebSocket ANTES de tentar conectar no robô
    web_server_task = asyncio.create_task(start_web_server(robot))
    
    # Tenta conectar no robô em background (não bloqueia o WebSocket)
    asyncio.create_task(robot.connect_robot())
    
    logging.info("⏸️ Aguardando comandos /play, /stop ou /status...")
    await web_server_task  # Mantém o servidor rodando
    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário.")
        sys.exit(0)
