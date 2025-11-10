# client.py
import asyncio, json, os, time
import websockets

class Connector:

    def __init__(self,ip,port):
        self.UBUNTU_IP = os.getenv("UBUNTU_IP", ip)
        self.WS_PORT = int(os.getenv("WS_PORT", port))
        self.WS_URL = f"ws://{self.UBUNTU_IP}:{self.WS_PORT}"

    async def send_gesture(self,gesture: str):
        payload = json.dumps({"gesture": gesture})
        try:
            async with websockets.connect(self.WS_URL, ping_interval=10, ping_timeout=10) as ws:
                hello = await ws.recv()
                print("Server:", hello)
                await ws.send(payload)
                print("Sent:", payload)
                reply = await ws.recv()
                print("Reply:", reply)
        except Exception as e:
            print("Error:", e)


    def on_gesture(self,gesture_word: str):
        asyncio.run(self.send_gesture(gesture_word))

