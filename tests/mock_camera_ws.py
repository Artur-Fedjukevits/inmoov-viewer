#!/usr/bin/env python3
"""Mock robot video stream: ws://0.0.0.0:9091, each frame is one binary JPEG 640×480 message, 15 fps, q80.
A slow client skips frames (each client has a queue of 1 frame).
  pip install websockets pillow;  python3 mock_camera_ws.py [--port 9091] [--fps 15]
"""
import argparse, asyncio, io, time, math
from PIL import Image, ImageDraw
import websockets

ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=9091); ap.add_argument("--fps", type=float, default=15)
ap.add_argument("--quality", type=int, default=80); a = ap.parse_args()
clients = set()

def frame(n):
    im = Image.new("RGB", (640, 480), (30, 34, 40)); d = ImageDraw.Draw(im)
    for x in range(0, 640, 40): d.line([(x, 0), (x, 480)], fill=(60, 66, 76))
    for y in range(0, 480, 40): d.line([(0, y), (640, y)], fill=(60, 66, 76))
    t = n / a.fps; cx, cy = 320 + 200 * math.cos(t), 240 + 140 * math.sin(t)
    d.ellipse([cx - 40, cy - 40, cx + 40, cy + 40], fill=(230, 120, 40))
    d.text((12, 10), "MOCK OAK-D Lite  frame %d  %s" % (n, time.strftime("%H:%M:%S")), fill=(240, 240, 240))
    b = io.BytesIO(); im.save(b, "JPEG", quality=a.quality); return b.getvalue()

async def handler(ws):
    q = asyncio.Queue(maxsize=1); clients.add(q)
    print("client +", ws.remote_address)
    try:
        while True:
            await ws.send(await q.get())
    except websockets.ConnectionClosed:
        pass
    finally:
        clients.discard(q); print("client -", ws.remote_address)

async def producer():
    n = 0
    while True:
        t0 = time.monotonic(); data = frame(n); n += 1
        for q in list(clients):
            if q.full():
                q.get_nowait()          # slow client: drop the old frame
            q.put_nowait(data)
        await asyncio.sleep(max(0, 1 / a.fps - (time.monotonic() - t0)))

async def main():
    async with websockets.serve(handler, "0.0.0.0", a.port, max_size=None, compression=None):
        print("mock camera on ws://0.0.0.0:%d" % a.port); await producer()

asyncio.run(main())
