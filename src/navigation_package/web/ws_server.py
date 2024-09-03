import asyncio
import websockets

connected_clients = set()

async def handler(websocket, path):
    print("Client connected")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            print(f"Received message: {message}")  # Print the received message
            for client in connected_clients:
                if client != websocket:
                    await client.send(message)
    finally:
        print("Client disconnected")
        connected_clients.remove(websocket)

async def main():
    print("WebSocket server started on ws://localhost:9002")
    async with websockets.serve(handler, "localhost", 9002):
        await asyncio.Future()  # Run forever

asyncio.run(main())
