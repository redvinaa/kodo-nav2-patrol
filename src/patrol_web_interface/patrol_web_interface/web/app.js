const { useState, useEffect, useRef } = React;

const API = "";

function apiPost(path, body, loadingMsg, setState) {
  setState(loadingMsg);
  fetch(`${API}${path}`, {
    method: "POST",
    ...(body ? { headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) } : {}),
  })
    .then(r => r.json())
    .then(d => setState(d.message || "Done"))
    .catch(() => setState("Request failed"));
}

function letterbox(mapW, mapH, cw, ch) {
  const scale = Math.min(cw / mapW, ch / mapH);
  return { w: Math.round(mapW * scale), h: Math.round(mapH * scale) };
}

function worldToCanvas(wx, wy, info, W, H) {
  const px = (wx - info.origin[0]) / info.resolution;
  const py = info.height - (wy - info.origin[1]) / info.resolution;
  return [px * W / info.width, py * H / info.height];
}

function drawArrow(ctx, x, y, yaw, size, color) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, -size * 0.5);
  ctx.lineTo(-size * 0.6,  size * 0.5);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

function App() {
  const canvasRef     = useRef(null);
  const containerRef  = useRef(null);
  const mapImgRef     = useRef(null);
  const mapInfoRef    = useRef(null);
  const robotPoseRef  = useRef(null);
  const globalPathRef = useRef([]);
  const patrolStateRef = useRef({ state_name: "IDLE", current_waypoint_index: 0, n_waypoints: 0, route_name: "" });
  const waypointsRef  = useRef([]);
  const rafRef        = useRef(null);
  const targetSizeRef  = useRef({ w: 0, h: 0 });

  const [routes, setRoutes]             = useState([]);
  const [selectedRoute, setSelectedRoute] = useState("");
  const [mapLoaded, setMapLoaded]       = useState(false);
  const [statusMsg, setStatusMsg]       = useState("");
  const [displayState, setDisplayState] = useState(patrolStateRef.current);

  // Load map + routes once
  useEffect(() => {
    fetch(`${API}/api/map_info`)
      .then(r => r.json())
      .then(info => {
        mapInfoRef.current = info;
        const img = new Image();
        img.onload = () => { mapImgRef.current = img; setMapLoaded(true); };
        img.src = `${API}/map`;
      })
      .catch(() => setStatusMsg("Failed to load map"));

    fetch(`${API}/api/list_routes`)
      .then(r => r.json())
      .then(data => setRoutes(data.routes || []))
      .catch(() => setStatusMsg("Failed to load routes"));
  }, []);

  // Fit canvas to container preserving aspect ratio
  useEffect(() => {
    if (!containerRef.current) return;
    const obs = new ResizeObserver(() => {
      const info = mapInfoRef.current;
      if (!info || !containerRef.current) return;
      const { w, h } = letterbox(info.width, info.height,
        containerRef.current.clientWidth, containerRef.current.clientHeight);
      targetSizeRef.current = { w, h };
    });
    obs.observe(containerRef.current);
    return () => obs.disconnect();
  }, []);

  // Resize once map info arrives
  useEffect(() => {
    if (!mapLoaded || !containerRef.current || !canvasRef.current) return;
    const info = mapInfoRef.current;
    const { w, h } = letterbox(info.width, info.height,
      containerRef.current.clientWidth, containerRef.current.clientHeight);
    targetSizeRef.current = { w, h };
  }, [mapLoaded]);

  // WebSocket — write directly into refs, no setState for live data
  useEffect(() => {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === "robot_pose")   robotPoseRef.current  = msg;
      if (msg.type === "global_path")  globalPathRef.current = msg.points || [];
      if (msg.type === "patrol_state") {
        patrolStateRef.current = msg;
        setDisplayState(msg);
      }
    };
    ws.onerror = () => setStatusMsg("WS error");
    return () => ws.close();
  }, []);

  // Fetch waypoints when route changes
  useEffect(() => {
    if (!selectedRoute) { waypointsRef.current = []; return; }
    fetch(`${API}/api/get_route?name=${encodeURIComponent(selectedRoute)}`)
      .then(r => r.json())
      .then(data => { waypointsRef.current = data.waypoints || []; })
      .catch(() => setStatusMsg("Failed to load route waypoints"));
  }, [selectedRoute]);

  // requestAnimationFrame draw loop
  useEffect(() => {
    function draw() {
      rafRef.current = requestAnimationFrame(draw);
      const canvas = canvasRef.current;
      const img    = mapImgRef.current;
      const info   = mapInfoRef.current;
      if (!canvas || !img || !info || canvas.width === 0) return;

      const { w: tw, h: th } = targetSizeRef.current;
      if (tw > 0 && (canvas.width !== tw || canvas.height !== th)) {
        canvas.width  = tw;
        canvas.height = th;
      }

      const ctx = canvas.getContext("2d");
      const W = canvas.width, H = canvas.height;

      ctx.imageSmoothingEnabled = false;
      ctx.clearRect(0, 0, W, H);
      ctx.drawImage(img, 0, 0, W, H);

      const to = (wx, wy) => worldToCanvas(wx, wy, info, W, H);
      const ps = patrolStateRef.current;
      const currentIdx = ps.current_waypoint_index;
      const isRunning  = ps.state_name === "RUNNING";

      // Global path
      const path = globalPathRef.current;
      if (path.length > 1) {
        ctx.beginPath();
        const [x0, y0] = to(path[0].x, path[0].y);
        ctx.moveTo(x0, y0);
        for (let i = 1; i < path.length; i++) {
          const [xi, yi] = to(path[i].x, path[i].y);
          ctx.lineTo(xi, yi);
        }
        ctx.strokeStyle = "rgba(0,180,255,0.75)";
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Waypoints
      waypointsRef.current.forEach(wp => {
        const isCurrent = isRunning && wp.index === currentIdx;
        const [cx, cy] = to(wp.x, wp.y);
        ctx.beginPath();
        ctx.arc(cx, cy, 9, 0, 2 * Math.PI);
        ctx.fillStyle = isCurrent ? "rgba(50,220,80,0.92)" : "rgba(255,200,0,0.85)";
        ctx.fill();
        ctx.strokeStyle = isCurrent ? "#fff" : "#000";
        ctx.lineWidth   = isCurrent ? 2 : 1;
        ctx.stroke();
        ctx.fillStyle = "#000";
        ctx.font = "bold 9px monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(wp.index), cx, cy);
      });

      // Robot pose
      const pose = robotPoseRef.current;
      if (pose) {
        const [rx, ry] = to(pose.x, pose.y);
        drawArrow(ctx, rx, ry, pose.yaw, 13, "#ff3030");
      }
    }

    rafRef.current = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(rafRef.current);
  }, []);

  const handleStart = () => {
    if (!selectedRoute) return;
    apiPost("/api/start", { route_name: selectedRoute }, "Starting...", setStatusMsg);
  };

  const handlePause = () => {
    apiPost("/api/pause", null, "Pausing...", setStatusMsg);
  };

  const handleResume = () => {
    apiPost("/api/resume", null, "Resuming...", setStatusMsg);
  };

  const handleStop = () => {
    apiPost("/api/stop", null, "Stopping...", setStatusMsg);
  };

  const stateClass = `state-${displayState.state_name}`;

  return (
    <div id="root">
      <div className="toolbar">
        <h1>Patrol Control</h1>
        <select value={selectedRoute} onChange={e => setSelectedRoute(e.target.value)}>
          <option value="">-- select route --</option>
          {routes.map(r => <option key={r} value={r}>{r}</option>)}
        </select>
        <button className="btn-start"  onClick={handleStart}  disabled={!selectedRoute || displayState.state_name === "RUNNING" || displayState.state_name === "PAUSED"}>Start</button>
        <button className="btn-pause"  onClick={handlePause}  disabled={displayState.state_name === "IDLE"}>Pause</button>
        <button className="btn-resume" onClick={handleResume} disabled={displayState.state_name === "IDLE"}>Resume</button>
        <button className="btn-stop"   onClick={handleStop}   disabled={displayState.state_name === "IDLE"}>Stop</button>
        <div className="status-bar">
          <span>State: <b className={stateClass}>{displayState.state_name}</b></span>
          {displayState.route_name && <span>Route: <b>{displayState.route_name}</b></span>}
          {displayState.n_waypoints > 0 && <span>WP: {displayState.current_waypoint_index}/{displayState.n_waypoints}</span>}
          {statusMsg && <span style={{color:"#aaa"}}>| {statusMsg}</span>}
        </div>
      </div>

      <div className="map-container" ref={containerRef}>
        <canvas ref={canvasRef} />
        {!mapLoaded && <div className="msg">Loading map...</div>}
        <div className="legend">
          <div><span style={{background:"rgba(255,200,0,0.85)",borderRadius:"50%"}}></span> Waypoint</div>
          <div><span style={{background:"rgba(50,220,80,0.92)",borderRadius:"50%"}}></span> Target WP</div>
          <div><span style={{background:"rgba(0,180,255,0.75)"}}></span> Global path</div>
          <div><span style={{background:"#ff3030"}}></span> Robot</div>
        </div>
      </div>
    </div>
  );
}

ReactDOM.createRoot(document.getElementById("root")).render(<App />);
