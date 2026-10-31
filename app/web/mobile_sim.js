// mobile_sim のブラウザ画面
//
// juiz の HTTP ブローカーの /api/container_process/call で、コンテナ mobile_sim の ContainerProcess を呼ぶ。
// 使うもの: get_map(画像)・get_map_info・get_state(10 Hz)・set_velocity(10 Hz)・reset_pose
"use strict";

const POLL_MS = 100;
const TRAIL_MAX = 2000;

// ---- juiz の呼び出し ----

class MobileSimClient {
  constructor(containerName) {
    this.containerName = containerName; // null なら最初に見つかった mobile_sim
    this.ids = {};
  }

  // ContainerProcess の一覧から、mobile_sim_* の識別子を集める
  async discover() {
    const res = await fetch("/api/container_process/list");
    if (!res.ok) throw new Error(`container_process/list: ${res.status}`);
    const list = await res.json();
    for (const e of list) {
      const [type, container] = e.type_name.split(":");
      if (!type.startsWith("mobile_sim_")) continue;
      if (this.containerName === null) this.containerName = container;
      if (container !== this.containerName) continue;
      const kind = type.slice("mobile_sim_".length);
      const ns = e.namespace ? `${e.namespace}/` : "";
      this.ids[kind] = `${e.broker_type_name}://${e.broker_name}/container_process/${ns}${e.name}::${e.type_name}`;
    }
    for (const k of ["get_state", "get_map", "get_map_info", "set_velocity", "reset_pose"]) {
      if (!this.ids[k]) throw new Error(`mobile_sim_${k} が見つかりません`);
    }
  }

  async call(kind, args = {}, asBlob = false) {
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[kind]);
    const res = await fetch(url, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(args),
    });
    if (!res.ok) throw new Error(`${kind}: ${res.status} ${await res.text()}`);
    return asBlob ? res.blob() : res.json();
  }

  state() { return this.call("get_state"); }
  mapInfo() { return this.call("get_map_info"); }
  mapImage() { return this.call("get_map", {}, true); }
  setVelocity(vx, omega, index = 0) {
    return this.call("set_velocity", { velocity: { linear: { x: vx, y: 0.0 }, angular: omega }, index });
  }
  resetPose(x, y, theta, index = 0) {
    return this.call("reset_pose", { pose: { position: { x, y }, theta }, index });
  }
}

// ---- 座標変換(マップは mapconf の origin が左下の画素の角) ----

class MapView {
  constructor(canvas, info, image) {
    this.canvas = canvas;
    this.info = info;
    this.image = image;
    const maxSide = 640;
    this.scale = Math.min(maxSide / info.width, maxSide / info.height); // [canvas px / map px]
    canvas.width = Math.round(info.width * this.scale);
    canvas.height = Math.round(info.height * this.scale);
    this.pxPerM = this.scale / info.resolution;
  }

  toCanvas(x, y) {
    const [ox, oy] = this.info.origin;
    return [(x - ox) * this.pxPerM, this.canvas.height - (y - oy) * this.pxPerM];
  }

  toWorld(cx, cy) {
    const [ox, oy] = this.info.origin;
    return [ox + cx / this.pxPerM, oy + (this.canvas.height - cy) / this.pxPerM];
  }
}

// ---- 描画 ----

function draw(view, state, trail, opts) {
  const ctx = view.canvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(view.image, 0, 0, view.canvas.width, view.canvas.height);

  if (opts.showTrail && trail.length > 1) {
    ctx.strokeStyle = "rgba(30, 110, 220, 0.6)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach(([x, y], i) => {
      const [cx, cy] = view.toCanvas(x, y);
      if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  for (const r of state.robots) {
    drawScan(ctx, view, r, opts.showRays);
    drawRobot(ctx, view, r);
  }
}

function drawScan(ctx, view, r, showRays) {
  const s = r.scan;
  // センサの位置 = ロボットの姿勢に取り付け位置を重ねたもの
  const c = Math.cos(r.pose.theta), sn = Math.sin(r.pose.theta);
  const sx = r.pose.x + c * s.mount.x - sn * s.mount.y;
  const sy = r.pose.y + sn * s.mount.x + c * s.mount.y;
  const st = r.pose.theta + s.mount.theta;
  const [scx, scy] = view.toCanvas(sx, sy);
  ctx.lineWidth = 1;
  s.ranges.forEach((range, i) => {
    const hit = range <= s.range_max;
    const d = hit ? range : s.range_max;
    const a = st + s.angle_min + i * s.angle_increment;
    const [ex, ey] = view.toCanvas(sx + d * Math.cos(a), sy + d * Math.sin(a));
    if (showRays) {
      ctx.strokeStyle = hit ? "rgba(255, 80, 0, 0.15)" : "rgba(0, 0, 0, 0.05)";
      ctx.beginPath();
      ctx.moveTo(scx, scy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
    }
    if (hit) {
      ctx.fillStyle = "#ff5000";
      ctx.fillRect(ex - 1.5, ey - 1.5, 3, 3);
    }
  });
}

function drawRobot(ctx, view, r) {
  const [cx, cy] = view.toCanvas(r.pose.x, r.pose.y);
  const rad = Math.max(r.radius * view.pxPerM, 4);
  ctx.fillStyle = r.bumper ? "rgba(220, 0, 0, 0.8)" : "rgba(0, 160, 80, 0.8)";
  ctx.strokeStyle = "#063";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  // 向き(キャンバスは y が下向きなので角度の符号を反転)
  ctx.strokeStyle = "#fff";
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + rad * Math.cos(-r.pose.theta), cy + rad * Math.sin(-r.pose.theta));
  ctx.stroke();
}

// ---- 速度の入力(キーボードとパッド) ----

class VelocityInput {
  constructor(pad) {
    this.keys = new Set();
    this.padValue = null; // [前後 -1..1, 旋回 -1..1]
    this.pad = pad;
    window.addEventListener("keydown", (e) => {
      if (e.target instanceof HTMLInputElement) return;
      if (this.keyAxis(e.key)) { this.keys.add(e.key); e.preventDefault(); }
    });
    window.addEventListener("keyup", (e) => this.keys.delete(e.key));
    window.addEventListener("blur", () => this.keys.clear());
    const move = (e) => {
      const rect = pad.getBoundingClientRect();
      const u = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      const v = ((e.clientY - rect.top) / rect.height) * 2 - 1;
      this.padValue = [clamp(-v, -1, 1), clamp(-u, -1, 1)];
    };
    pad.addEventListener("pointerdown", (e) => { pad.setPointerCapture(e.pointerId); move(e); });
    pad.addEventListener("pointermove", (e) => { if (this.padValue) move(e); });
    const release = () => { this.padValue = null; };
    pad.addEventListener("pointerup", release);
    pad.addEventListener("pointercancel", release);
    this.drawPad();
  }

  keyAxis(key) {
    return {
      w: [1, 0], ArrowUp: [1, 0], s: [-1, 0], ArrowDown: [-1, 0],
      a: [0, 1], ArrowLeft: [0, 1], d: [0, -1], ArrowRight: [0, -1],
    }[key];
  }

  // [前後, 旋回] をそれぞれ -1..1 で返す。入力が無ければ null
  value() {
    if (this.padValue) return this.padValue;
    if (this.keys.size === 0) return null;
    let f = 0, t = 0;
    for (const k of this.keys) {
      const ax = this.keyAxis(k);
      if (ax) { f += ax[0]; t += ax[1]; }
    }
    return [clamp(f, -1, 1), clamp(t, -1, 1)];
  }

  drawPad() {
    const ctx = this.pad.getContext("2d");
    const { width: w, height: h } = this.pad;
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = "#bbb";
    ctx.beginPath();
    ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h);
    ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
    ctx.stroke();
    const v = this.value();
    if (v) {
      ctx.fillStyle = "#2d6ad6";
      ctx.beginPath();
      ctx.arc(w / 2 - (v[1] * w) / 2, h / 2 - (v[0] * h) / 2, 10, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

// ---- 初期位置の指定(クリック、ドラッグで向き) ----

function setupPlacement(view, client, getState, onPlaced) {
  const canvas = view.canvas;
  let start = null;
  const pos = (e) => {
    const rect = canvas.getBoundingClientRect();
    return [
      ((e.clientX - rect.left) * canvas.width) / rect.width,
      ((e.clientY - rect.top) * canvas.height) / rect.height,
    ];
  };
  canvas.addEventListener("pointerdown", (e) => { canvas.setPointerCapture(e.pointerId); start = pos(e); });
  canvas.addEventListener("pointerup", async (e) => {
    if (!start) return;
    const end = pos(e);
    const [x, y] = view.toWorld(start[0], start[1]);
    let theta;
    if (Math.hypot(end[0] - start[0], end[1] - start[1]) > 5) {
      theta = Math.atan2(-(end[1] - start[1]), end[0] - start[0]);
    } else {
      const s = getState();
      theta = s ? s.robots[0].pose.theta : 0.0;
    }
    start = null;
    try { await client.resetPose(x, y, theta); onPlaced(); } catch (err) { showError(err); }
  });
}

// ---- 表示 ----

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => v.toFixed(d);

function showState(state) {
  const r = state.robots[0];
  $("s_time").textContent = `${fmt(state.time, 1)} s`;
  $("s_pose").textContent = `x ${fmt(r.pose.x)}, y ${fmt(r.pose.y)}, θ ${fmt(r.pose.theta)}`;
  $("s_vel").textContent = `v ${fmt(r.velocity.vx)}, ω ${fmt(r.velocity.omega)}`;
  $("s_cmd").textContent = `v ${fmt(r.target_velocity.vx)}, ω ${fmt(r.target_velocity.omega)}`;
  $("s_bumper").textContent = r.bumper ? "当たっている" : "なし";
  $("s_bumper").classList.toggle("hit", r.bumper);
}

function showError(err) {
  console.error(err);
  const c = $("connection");
  c.textContent = `エラー: ${err.message}`;
  c.className = "status error";
}

function showConnected(name) {
  const c = $("connection");
  c.textContent = `接続済み(コンテナ ${name})`;
  c.className = "status ok";
}

async function loadImage(blob) {
  const url = URL.createObjectURL(blob);
  const img = new Image();
  img.src = url;
  await img.decode();
  return img;
}

async function main() {
  const params = new URLSearchParams(location.search);
  const client = new MobileSimClient(params.get("container"));
  await client.discover();
  showConnected(client.containerName);

  const view = new MapView($("map"), await client.mapInfo(), await loadImage(await client.mapImage()));
  const input = new VelocityInput($("pad"));
  let state = null;
  let trail = [];
  let sentZero = true;
  setupPlacement(view, client, () => state, () => { trail = []; });
  $("clear_trail").addEventListener("click", () => { trail = []; });
  $("stop").addEventListener("click", () => client.setVelocity(0, 0).catch(showError));

  setInterval(async () => {
    try {
      state = await client.state();
      const p = state.robots[0].pose;
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.02) {
        trail.push([p.x, p.y]);
        if (trail.length > TRAIL_MAX) trail.shift();
      }
      draw(view, state, trail, { showRays: $("show_rays").checked, showTrail: $("show_trail").checked });
      showState(state);

      // 入力があるあいだは指令を送り続ける(シミュレータは指令が途切れると止まる)
      input.drawPad();
      const v = input.value();
      if (v) {
        await client.setVelocity(v[0] * Number($("max_v").value), v[1] * Number($("max_w").value));
        sentZero = false;
      } else if (!sentZero) {
        await client.setVelocity(0, 0);
        sentZero = true;
      }
      showConnected(client.containerName);
    } catch (err) {
      showError(err);
    }
  }, POLL_MS);
}

main().catch(showError);
