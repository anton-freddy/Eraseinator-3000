import asyncio
import websockets

async def handler(websocket, path):
    while True:
        data = await websocket.recv()
        print(f"Received data: {data}")

async def main():
    # Replace with the actual IP address and port of your computer
    server_ip = "0.0.0.0"  # Use 0.0.0.0 to listen on all available interfaces
    server_port = 8765

    uri = f"ws://{server_ip}:{server_port}"

    async with websockets.serve(handler, server_ip, server_port):
        print(f"WebSocket server started at {uri}")
        await asyncio.Future()  # Run the server indefinitely

if __name__ == "__main__":
    asyncio.run(main())
