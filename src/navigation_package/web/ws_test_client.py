import asyncio
import websockets

async def test():
    uri = "ws://localhost:9002"
    async with websockets.connect(uri) as websocket:
        await websocket.send("Hello Server!")
        response = await websocket.recv()
        print(f"Received: {response}")

asyncio.run(test())
