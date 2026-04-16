"""
web_server.py - FastAPI HTTP + WebSocket bridge between ROS 2 and the React UI.

Endpoints
---------
GET  /map                      - Map image (PNG)
GET  /api/map_info             - Map metadata (resolution, origin, size)
GET  /api/list_routes          - Calls patrol/list_routes ROS service
GET  /api/get_route?name=<n>   - Calls patrol/get_route ROS service
POST /api/start                - Calls patrol/start  (body: {"route_name": "..."})
POST /api/stop                 - Calls patrol/stop
WS   /ws                       - Streams patrol state, robot pose, global path as JSON
"""

from __future__ import annotations

import asyncio
import io
import math
import struct
import threading
import zlib
from pathlib import Path
from typing import Optional, Set

import rclpy
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from patrol_interfaces.msg import PatrolState
from patrol_interfaces.srv import GetRoute, ListRoutes, StartPatrol
from pydantic import BaseModel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


# WebSocket client registry
_ws_clients: Set[WebSocket] = set()
_ws_lock = asyncio.Lock()

# Latest data from ROS callbacks (written by ROS thread, read by asyncio)
_latest_state: Optional[dict] = None
_latest_path: Optional[dict] = None
_latest_pose: Optional[dict] = None

# asyncio event loop (set in main)
_loop: Optional[asyncio.AbstractEventLoop] = None

# Events created at module level so they are never None when ROS callbacks fire.
# main() reassigns them to new Event() instances bound to the correct loop.
_state_event: asyncio.Event = asyncio.Event()
_path_event: asyncio.Event = asyncio.Event()
_pose_event: asyncio.Event = asyncio.Event()
_any_event: asyncio.Event = asyncio.Event()


def _trigger(event: asyncio.Event) -> None:
    """Thread-safe: set a per-topic event and the shared wake event from a ROS thread."""
    if _loop is not None:
        _loop.call_soon_threadsafe(event.set)
        _loop.call_soon_threadsafe(_any_event.set)


class WebBridgeNode(Node):
    """Subscribes to ROS topics and wraps ROS service calls."""

    def __init__(self, routes_dir: str) -> None:
        super().__init__("patrol_web_server")
        self._routes_dir = Path(routes_dir)

        self.create_subscription(PatrolState, "patrol/state", self._on_patrol_state, 10)
        self.create_subscription(NavPath, "/plan", self._on_global_path, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )

        self._list_routes_client = self.create_client(ListRoutes, "patrol/list_routes")
        self._get_route_client = self.create_client(GetRoute, "patrol/get_route")
        self._start_client = self.create_client(StartPatrol, "patrol/start")
        self._stop_client = self.create_client(Trigger, "patrol/stop")
        self._pause_client = self.create_client(Trigger, "patrol/pause")
        self._resume_client = self.create_client(Trigger, "patrol/resume")

    # ROS topic callbacks

    def _on_patrol_state(self, msg: PatrolState) -> None:
        global _latest_state
        _latest_state = {
            "type": "patrol_state",
            "state": msg.state,
            "state_name": _state_name(msg.state),
            "route_name": msg.route_name,
            "current_waypoint_index": msg.current_waypoint_index,
            "n_waypoints": msg.n_waypoints,
        }
        _trigger(_state_event)

    def _on_global_path(self, msg: NavPath) -> None:
        global _latest_path
        _latest_path = {
            "type": "global_path",
            "points": [
                {"x": p.pose.position.x, "y": p.pose.position.y}
                for p in msg.poses
            ],
        }
        _trigger(_path_event)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        global _latest_pose
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z),
        )
        _latest_pose = {"type": "robot_pose", "x": pos.x, "y": pos.y, "yaw": yaw}
        _trigger(_pose_event)

    # Synchronous ROS service calls (run via run_in_executor).
    # Use a threading.Event to wait for the result without blocking the executor.

    def _wait_for_future(self, future, timeout_sec: float = 5.0):
        """Block the calling thread until a ROS future completes, without spin."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_sec):
            raise RuntimeError("Service call timed out")
        return future.result()

    def call_list_routes(self) -> list:
        if not self._list_routes_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/list_routes not available")
        result = self._wait_for_future(self._list_routes_client.call_async(ListRoutes.Request()))
        if result is None:
            raise RuntimeError("No response from patrol/list_routes")
        return list(result.route_names)

    def call_get_route(self, route_name: str) -> list:
        if not self._get_route_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/get_route not available")
        req = GetRoute.Request()
        req.route_name = route_name
        result = self._wait_for_future(self._get_route_client.call_async(req))
        if result is None:
            raise RuntimeError("No response from patrol/get_route")
        if not result.success:
            raise RuntimeError(result.message)
        return [
            {"index": i, "x": float(wp.x), "y": float(wp.y), "yaw": float(wp.theta)}
            for i, wp in enumerate(result.waypoints)
        ]

    def call_start(self, route_name: str) -> dict:
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/start not available")
        req = StartPatrol.Request()
        req.route_name = route_name
        result = self._wait_for_future(self._start_client.call_async(req))
        if result is None:
            raise RuntimeError("No response from patrol/start")
        return {"success": result.success, "message": result.message}

    def call_stop(self) -> dict:
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/stop not available")
        result = self._wait_for_future(self._stop_client.call_async(Trigger.Request()))
        if result is None:
            raise RuntimeError("No response from patrol/stop")
        return {"success": result.success, "message": result.message}

    def call_pause(self) -> dict:
        if not self._pause_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/pause not available")
        result = self._wait_for_future(self._pause_client.call_async(Trigger.Request()))
        if result is None:
            raise RuntimeError("No response from patrol/pause")
        return {"success": result.success, "message": result.message}

    def call_resume(self) -> dict:
        if not self._resume_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("patrol/resume not available")
        result = self._wait_for_future(self._resume_client.call_async(Trigger.Request()))
        if result is None:
            raise RuntimeError("No response from patrol/resume")
        return {"success": result.success, "message": result.message}


# Helpers

def _state_name(state: int) -> str:
    return {0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "COMPLETED", 4: "FAILED"}.get(
        state, "UNKNOWN"
    )


def _find_map_file(filename: str) -> Optional[Path]:
    candidates = [
        Path(f"/ros2_ws/src/patrol_navigation/maps/{filename}"),
        Path(f"/ros2_ws/install/patrol_navigation/share/patrol_navigation/maps/{filename}"),
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def _pgm_to_png(pgm_path: Path) -> bytes:
    """Convert a PGM file to PNG bytes. Uses Pillow if available, else stdlib."""
    try:
        from PIL import Image  # type: ignore
        img = Image.open(str(pgm_path))
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()
    except ImportError:
        pass

    with open(pgm_path, "rb") as f:
        content = f.read()

    lines = []
    idx = 0
    while len(lines) < 3:
        end = content.index(b"\n", idx)
        line = content[idx:end].decode("ascii").strip()
        idx = end + 1
        if line.startswith("#"):
            continue
        lines.append(line)

    magic = lines[0]
    width, height = map(int, lines[1].split())
    maxval = int(lines[2])
    raw = content[idx:] if magic == "P5" else bytes(int(v) for v in content[idx:].split())

    if maxval != 255:
        raw = bytes(int(b * 255 / maxval) for b in raw)

    def chunk(name: bytes, data: bytes) -> bytes:
        crc = struct.pack(">I", zlib.crc32(name + data) & 0xFFFFFFFF)
        return struct.pack(">I", len(data)) + name + data + crc

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    raw_rows = b"".join(
        b"\x00" + raw[r * width: (r + 1) * width] for r in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw_rows))
        + chunk(b"IEND", b"")
    )


# FastAPI app

app = FastAPI()
_node: Optional[WebBridgeNode] = None


@app.get("/map")
async def get_map():
    pgm = _find_map_file("map.pgm")
    if pgm is None:
        raise HTTPException(status_code=404, detail="map.pgm not found")
    png_bytes = await _loop.run_in_executor(None, _pgm_to_png, pgm)
    return Response(content=png_bytes, media_type="image/png")


@app.get("/api/map_info")
async def get_map_info():
    yaml_path = _find_map_file("map.yaml")
    if yaml_path is None:
        raise HTTPException(status_code=404, detail="map.yaml not found")
    with open(yaml_path, "r") as fh:
        meta = yaml.safe_load(fh)

    width, height = 0, 0
    pgm_path = yaml_path.parent / meta["image"]
    if pgm_path.exists():
        with open(pgm_path, "rb") as f:
            lines = []
            while len(lines) < 2:
                line = f.readline().decode("ascii").strip()
                if not line.startswith("#") and line:
                    lines.append(line)
            if len(lines) >= 2:
                try:
                    width, height = map(int, lines[1].split())
                except ValueError:
                    pass

    return {
        "resolution": meta.get("resolution", 0.05),
        "origin": meta.get("origin", [-10.0, -10.0, 0.0]),
        "width": width,
        "height": height,
        "negate": meta.get("negate", 0),
    }


@app.get("/api/list_routes")
async def list_routes():
    try:
        routes = await _loop.run_in_executor(None, _node.call_list_routes)
        return {"routes": routes}
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.get("/api/get_route")
async def get_route(name: str = ""):
    if not name:
        raise HTTPException(status_code=400, detail="missing 'name' query param")
    try:
        waypoints = await _loop.run_in_executor(None, _node.call_get_route, name)
        return {"route_name": name, "waypoints": waypoints}
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


class StartRequest(BaseModel):
    route_name: str


@app.post("/api/start")
async def start_patrol(body: StartRequest):
    if not body.route_name:
        raise HTTPException(status_code=400, detail="missing 'route_name'")
    try:
        result = await _loop.run_in_executor(None, _node.call_start, body.route_name)
        return result
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.post("/api/stop")
async def stop_patrol():
    try:
        result = await _loop.run_in_executor(None, _node.call_stop)
        return result
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.post("/api/pause")
async def pause_patrol():
    try:
        result = await _loop.run_in_executor(None, _node.call_pause)
        return result
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.post("/api/resume")
async def resume_patrol():
    try:
        result = await _loop.run_in_executor(None, _node.call_resume)
        return result
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    async with _ws_lock:
        _ws_clients.add(websocket)

    try:
        # Send latest known values immediately on connect
        for payload in (_latest_state, _latest_pose, _latest_path):
            if payload is not None:
                await websocket.send_json(payload)
        # Block here until the client disconnects; all pushes come from _broadcast_loop
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        async with _ws_lock:
            _ws_clients.discard(websocket)


async def _broadcast_loop() -> None:
    """Push ROS updates to all connected WebSocket clients."""
    while True:
        await _any_event.wait()
        _any_event.clear()

        messages = []
        for event, latest in (
            (_state_event, _latest_state),
            (_path_event, _latest_path),
            (_pose_event, _latest_pose),
        ):
            if event.is_set():
                event.clear()
                if latest is not None:
                    messages.append(latest)

        if messages:
            async with _ws_lock:
                global _ws_clients
                dead = set()
                for ws in _ws_clients:
                    for msg in messages:
                        try:
                            await ws.send_json(msg)
                        except Exception:
                            dead.add(ws)
                            break
                _ws_clients -= dead


@app.on_event("startup")
async def on_startup():
    asyncio.ensure_future(_broadcast_loop())


def main(args=None) -> None:
    global _node, _loop

    rclpy.init(args=args)

    _node = WebBridgeNode(routes_dir="/ros2_ws/routes")
    _node.declare_parameter("host", "0.0.0.0")
    _node.declare_parameter("port", 8080)
    _node.declare_parameter("routes_dir", "/ros2_ws/routes")
    _node.declare_parameter("web_dir", "")

    host = _node.get_parameter("host").get_parameter_value().string_value
    port = _node.get_parameter("port").get_parameter_value().integer_value
    web_dir_param = _node.get_parameter("web_dir").get_parameter_value().string_value

    executor = MultiThreadedExecutor()
    executor.add_node(_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Serve React build from web_dir parameter (resolved by launch)
    # Mount on /ui to avoid shadowing /ws and /api routes
    web_dir = Path(web_dir_param) if web_dir_param else None
    if web_dir and web_dir.exists():
        app.mount("/", StaticFiles(directory=str(web_dir), html=True), name="static")
    else:
        _node.get_logger().warning(f"Web directory not found or not set: '{web_dir_param}'")

    _loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_loop)

    config = uvicorn.Config(app, host=host, port=port, loop="none", log_level="info")
    server = uvicorn.Server(config)

    try:
        _loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        _loop.close()


if __name__ == "__main__":
    main()
