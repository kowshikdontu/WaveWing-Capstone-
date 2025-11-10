import asyncio
import json
import logging
import threading
import queue
from typing import Set, Optional, Literal, Dict, Callable

import websockets
from websockets import WebSocketServerProtocol
from dronekit import connect, VehicleMode

from basic_func import MotionTracker, DroneController

HOST = "0.0.0.0"
PORT = 8765

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("gesture-ws")


vehicle = connect("127.0.0.1:14550", wait_ready=True)
motion_tracker = MotionTracker(vehicle, 0.1)
controller = DroneController(motion_tracker, vehicle)
vehicle.mode = VehicleMode("STABILIZE")
vehicle.parameters["LOG_DISARMED"] = 1
vehicle.parameters["ARMING_CHECK"] = 1


Gesture = Literal["TAKEOFF", "LAND", "Left", "Right", "Forward", "Backward"]

NON_REPEATABLE: Set[Gesture] = {"TAKEOFF", "LAND","Left", "Right", "Forward", "Backward"}
REPEATABLE: Set[Gesture] = {}

class CommandQueue:
    def __init__(self, maxsize: int = 1000):
        self.q: "queue.Queue[Gesture]" = queue.Queue(maxsize=maxsize)
        self._last_enqueued: Optional[Gesture] = None
        self._lock = threading.Lock()
        self.in_air = False

    def put(self, cmd: Gesture) -> bool:
        """
        Enqueue with de-dupe rule:
        - If cmd in NON_REPEATABLE and same as last_enqueued => drop (return False)
        - Else put (return True)
        """
        with self._lock:
            if cmd in NON_REPEATABLE and self._last_enqueued == cmd and not self.in_air:
                return False
            try:
                if self._last_enqueued == cmd:
                    return False
                self.q.put_nowait(cmd)
                self._last_enqueued = cmd
                return True
            except queue.Full:
                logger.warning("Command queue full; dropping %s", cmd)
                return False

    def get(self) -> Gesture:
        return self.q.get()

    def task_done(self) -> None:
        self.q.task_done()

cmd_queue = CommandQueue()

def _exec_takeoff():

    controller.arm_and_takeoff(10)

def _exec_land():
    # If you have a true land mode, call it. Here we step down as in your code.
    controller.move_down(1)

def _exec_left():
    controller.move_left(10)

def _exec_right():
    controller.move_right(10)

def _exec_forward():
    controller.move_forward(10)

def _exec_backward():
    controller.move_backward(10)

def _exec_unknown():
    controller.send_velocity(0, 0, 0)

EXEC_MAP: Dict[str, Callable[[], None]] = {
    "TAKEOFF": _exec_takeoff,
    "LAND": _exec_land,
    "Left": _exec_left,
    "Right": _exec_right,
    "Forward": _exec_forward,
    "Backward": _exec_backward,
}

def worker_loop():

    logger.info("Command worker started")
    while True:
        cmd = cmd_queue.get()
        try:
            if cmd_queue.in_air or cmd=="TAKEOFF":
                fn = EXEC_MAP.get(cmd, _exec_unknown)
                fn()  # blocking call to DroneKit/controller
            if cmd=="TAKEOFF":
                cmd_queue.in_air=True
            logger.info("Executing command: %s", cmd)

        except Exception:
            logger.exception("Error while executing command: %s", cmd)
        finally:
            cmd_queue.task_done()

worker = threading.Thread(target=worker_loop, name="gesture-worker", daemon=True)
worker.start()



CONNECTED: Set[WebSocketServerProtocol] = set()

async def register(ws: WebSocketServerProtocol):
    CONNECTED.add(ws)
    logger.info("Client connected %s (total %d)", ws.remote_address, len(CONNECTED))

async def unregister(ws: WebSocketServerProtocol):
    CONNECTED.discard(ws)
    logger.info("Client disconnected %s (total %d)", ws.remote_address, len(CONNECTED))

def enqueue_gesture(gesture: str) -> str:
    """
    Validate + enqueue using plain queue rules.
    Returns result message string for the ws ack.
    """
    g = gesture.strip()
    # Accept only known gestures; unknown goes to _exec_unknown but still OK to enqueue?
    # If you prefer to reject unknowns, flip the logic here.
    if g not in EXEC_MAP:
        # Enqueue a "noop"/stop to zero velocity to be safe
        enq_ok = cmd_queue.put("UNKNOWN") if "UNKNOWN" in EXEC_MAP else True
        return "OK: unknown gesture received; sent stop" if enq_ok else "ERROR: queue full"

    enq_ok = cmd_queue.put(g)  # applies de-dupe for NON_REPEATABLE
    if not enq_ok:
        if g in NON_REPEATABLE:
            return f"OK: duplicate {g} dropped"
        return "ERROR: queue full"
    return f"OK: queued {g}"

async def ws_handler(ws: WebSocketServerProtocol, path: str):
    await register(ws)
    try:
        await ws.send("Hello from Ubuntu WS server!")
        async for message in ws:
            logger.info("Received raw: %s from %s", message, ws.remote_address)
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                await ws.send(json.dumps({"status": "error", "detail": "invalid json"}))
                continue

            gesture = data.get("gesture")
            if not gesture:
                await ws.send(json.dumps({"status": "error", "detail": "missing 'gesture' field"}))
                continue

            try:
                result = enqueue_gesture(gesture)
            except Exception as e:
                logger.exception("enqueue_gesture error")
                await ws.send(json.dumps({"status": "error", "detail": str(e)}))
                continue

            await ws.send(json.dumps({"status": "ok", "gesture": gesture, "result": result}))
    except websockets.exceptions.ConnectionClosedOK:
        logger.info("Connection closed normally")
    except Exception:
        logger.exception("WebSocket server error")
    finally:
        await unregister(ws)

async def main():
    logger.info("Starting server on ws://%s:%d", HOST, PORT)
    async with websockets.serve(ws_handler, HOST, PORT, ping_interval=60, ping_timeout=60):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server shutting down")
