// IP debug UI — websocket client + canvas renderers + keyboard teleop.
"use strict";

const $ = id => document.getElementById(id);
const cvRGB = $("cv-rgb"), ctxRGB = cvRGB.getContext("2d");
const cvMask = $("cv-mask"), ctxMask = cvMask.getContext("2d");
const cvPCD = $("cv-pcd"), ctxPCD = cvPCD.getContext("2d");
const stateReadout = $("state-readout");
const status = $("status");
const logEl = $("log");

const wsUrl = (location.protocol === "https:" ? "wss:" : "ws:")
  + "//" + location.host + "/ws";
let ws = null;
let lastFrameTs = 0, frameCount = 0, fps = 0;

const COLORS = ["#fa6", "#6f8", "#6cf", "#fc6", "#f88", "#8c8", "#88f", "#cfc"];

function colorForLabel(label, idx) {
  return COLORS[idx % COLORS.length];
}

// ---------- log ----------
const MAX_LOG = 200;
function logLine(text, klass = "") {
  const div = document.createElement("div");
  div.className = "log-line " + klass;
  const ts = new Date();
  const t = ts.toTimeString().slice(0, 8) + "."
    + String(ts.getMilliseconds()).padStart(3, "0");
  div.innerHTML = `<span class="t">${t}</span>${text}`;
  logEl.prepend(div);
  while (logEl.childElementCount > MAX_LOG) logEl.removeChild(logEl.lastChild);
}

// ---------- canvas drawers ----------
function drawRGB(rgbDataURL, boxes, labels, scores) {
  const img = new Image();
  img.onload = () => {
    ctxRGB.drawImage(img, 0, 0, 640, 480);
    if (Array.isArray(boxes)) {
      boxes.forEach((b, i) => {
        ctxRGB.strokeStyle = colorForLabel(labels[i], i);
        ctxRGB.lineWidth = 2;
        const [x1, y1, x2, y2] = b.map(v => Math.round(v));
        ctxRGB.strokeRect(x1, y1, x2 - x1, y2 - y1);
        ctxRGB.fillStyle = colorForLabel(labels[i], i);
        ctxRGB.font = "12px sans-serif";
        const txt = `${labels[i] || "?"} ${(scores?.[i] ?? 0).toFixed(2)}`;
        ctxRGB.fillText(txt, x1, Math.max(10, y1 - 3));
      });
    }
  };
  img.src = rgbDataURL;
}

function drawMask(rgbDataURL, maskB64) {
  const img = new Image();
  img.onload = () => {
    ctxMask.drawImage(img, 0, 0, 640, 480);
    if (!maskB64) return;
    const m = new Image();
    m.onload = () => {
      const off = document.createElement("canvas");
      off.width = 640; off.height = 480;
      const c = off.getContext("2d");
      c.drawImage(m, 0, 0, 640, 480);
      const data = c.getImageData(0, 0, 640, 480);
      const px = data.data;
      for (let i = 0; i < px.length; i += 4) {
        const v = px[i];
        if (v > 0) {
          px[i] = 30; px[i + 1] = 230; px[i + 2] = 80;
          px[i + 3] = 140;
        } else {
          px[i + 3] = 0;
        }
      }
      c.putImageData(data, 0, 0);
      ctxMask.drawImage(off, 0, 0);
    };
    m.src = "data:image/png;base64," + maskB64;
  };
  img.src = rgbDataURL;
}

function drawPCD(pcd, eePos) {
  const W = cvPCD.width, H = cvPCD.height;
  ctxPCD.fillStyle = "#0a0a0a";
  ctxPCD.fillRect(0, 0, W, H);
  ctxPCD.strokeStyle = "#222";
  ctxPCD.lineWidth = 1;
  for (let xb = 0; xb <= 0.9; xb += 0.1) {
    const px = (xb / 0.9) * W;
    ctxPCD.beginPath(); ctxPCD.moveTo(px, 0); ctxPCD.lineTo(px, H); ctxPCD.stroke();
  }
  for (let yb = -0.4; yb <= 0.4; yb += 0.1) {
    const py = ((0.4 - yb) / 0.8) * H;
    ctxPCD.beginPath(); ctxPCD.moveTo(0, py); ctxPCD.lineTo(W, py); ctxPCD.stroke();
  }
  ctxPCD.fillStyle = "#666"; ctxPCD.font = "10px monospace";
  ctxPCD.fillText("x_base = 0",   2, H - 4);
  ctxPCD.fillText("x_base = 0.9", W - 78, H - 4);
  ctxPCD.fillText("y_base = +0.4", 2, 12);
  ctxPCD.fillText("y_base = -0.4", 2, H - 14);
  const ox = 0;
  const oy = ((0.4 - 0) / 0.8) * H;
  ctxPCD.strokeStyle = "#fff";
  ctxPCD.beginPath();
  ctxPCD.moveTo(ox - 6, oy); ctxPCD.lineTo(ox + 6, oy);
  ctxPCD.moveTo(ox, oy - 6); ctxPCD.lineTo(ox, oy + 6);
  ctxPCD.stroke();
  if (Array.isArray(pcd)) {
    ctxPCD.fillStyle = "#5cf5a8";
    for (let i = 0; i < pcd.length; i++) {
      const p = pcd[i];
      const x = p[0], y = p[1];
      if (x < 0 || x > 0.9 || y < -0.4 || y > 0.4) continue;
      const px = (x / 0.9) * W;
      const py = ((0.4 - y) / 0.8) * H;
      ctxPCD.fillRect(px - 0.5, py - 0.5, 1.5, 1.5);
    }
  }
  if (Array.isArray(eePos)) {
    const x = eePos[0], y = eePos[1];
    if (x >= 0 && x <= 0.9 && y >= -0.4 && y <= 0.4) {
      const px = (x / 0.9) * W;
      const py = ((0.4 - y) / 0.8) * H;
      ctxPCD.fillStyle = "#ff7a7a";
      ctxPCD.beginPath();
      ctxPCD.arc(px, py, 6, 0, 2 * Math.PI);
      ctxPCD.fill();
      ctxPCD.fillStyle = "#fff"; ctxPCD.font = "10px monospace";
      ctxPCD.fillText("EE", px + 8, py + 4);
    }
  }
}

// ---------- state readout ----------
function fmt3(arr) { return "[" + arr.map(v => v.toFixed(3)).join(", ") + "]"; }

function updateState(msg) {
  const ee = msg.ee_pos || [0,0,0];
  const q  = msg.ee_quat || [0,0,0,1];
  const gw = msg.gripper_width;
  const segAge = msg.seg_age_ms;
  const recLine = msg.recording
    ? `RECORDING   ${(msg.recording_dur_s ?? 0).toFixed(1)} s, ${msg.recording_n} frames`
    : `recording   idle`;
  const ipLine = msg.running_ip
    ? `IP RUNNING  step ${msg.ip_step ?? 0}`
    : `IP run      idle`;
  stateReadout.textContent =
`ee_pos      ${fmt3(ee)}
ee_quat     ${fmt3(q.slice(0,3))} ${q[3].toFixed(3)}
gripper     ${gw === undefined ? "?" : (gw * 1000).toFixed(1) + " mm"}
boxes       ${msg.boxes ? msg.boxes.length : 0}
pcd_n       ${msg.pcd_n ?? "?"}
seg_age     ${segAge === undefined ? "?" : segAge.toFixed(0) + " ms"}
fps         ${fps.toFixed(1)}
${recLine}
${ipLine}`;
  // Reflect server-truth recording state on the toggle button (in case the
  // user reloaded the page mid-recording).
  const btn = $("btn-record");
  if (btn) {
    btn.classList.toggle("recording", !!msg.recording);
    btn.textContent = msg.recording
      ? `■ stop & save (${msg.recording_n})`
      : `● record`;
  }
  const ipBtn = $("btn-ip-run");
  if (ipBtn) {
    ipBtn.classList.toggle("running", !!msg.running_ip);
    ipBtn.textContent = msg.running_ip
      ? `■ stop IP (${msg.ip_step ?? 0})`
      : `▶ run IP`;
  }
}

// ---------- keyboard map highlight ----------
const kbKeys = Array.from(document.querySelectorAll(".kb-key"));
function highlightKey(key, on) {
  // Match by data-k. Special-case Shift/Control/Space.
  const norm = key.length === 1 ? key.toLowerCase() : key;
  for (const el of kbKeys) {
    const k = el.dataset.k;
    if (k === norm || (k && k.toLowerCase() === norm)) {
      el.classList.toggle("kb-active", on);
    }
  }
}

// ---------- websocket ----------
function connect() {
  ws = new WebSocket(wsUrl);
  ws.onopen = () => {
    status.textContent = "connected"; status.className = "ok";
    logLine("ws connected", "");
  };
  ws.onclose = () => {
    status.textContent = "disconnected — retrying"; status.className = "bad";
    logLine("ws disconnected, retrying in 1.5s", "warn");
    setTimeout(connect, 1500);
  };
  ws.onerror = (e) => console.warn("ws error", e);
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "config") {
      // Pre-fill input with the server-side default
      const inp = $("prompt-input");
      if (inp && msg.prompt) inp.value = msg.prompt;
      logLine(`prompt default: "${msg.prompt}"`, "");
      return;
    }
    if (msg.type === "frame") {
      const dataURL = "data:image/jpeg;base64," + msg.rgb_jpeg_b64;
      drawRGB(dataURL, msg.boxes, msg.box_labels, msg.box_scores);
      drawMask(dataURL, msg.mask_png_b64);
      drawPCD(msg.pcd_w, msg.ee_pos);
      updateState(msg);
      const now = performance.now();
      frameCount++;
      if (now - lastFrameTs > 500) {
        fps = frameCount * 1000 / (now - lastFrameTs);
        frameCount = 0; lastFrameTs = now;
      }
      return;
    }
    if (msg.type === "reset_ack") {
      logLine(`reset ack: ${JSON.stringify(msg.ack)}`, "");
      return;
    }
    if (msg.type === "record_started") {
      logLine(`recording started (name=${msg.name || "—"})`, "rec");
      return;
    }
    if (msg.type === "record_saved") {
      if (msg.ok) {
        logLine(`saved demo: ${msg.path}  buffer=${msg.buffer_n}`
          + ` waypoints=${msg.waypoints} grips=${JSON.stringify(msg.grips)}`,
          "rec");
      } else {
        logLine(`save FAILED: ${msg.msg}`, "warn");
      }
      return;
    }
    if (msg.type === "record_cancelled") {
      logLine("recording cancelled", "rec");
      return;
    }
    if (msg.type === "ip_run_started") {
      logLine("IP closed-loop START — keyboard teleop disabled", "rec");
      return;
    }
    if (msg.type === "ip_run_stopped") {
      logLine("IP closed-loop STOP — keyboard teleop re-enabled", "rec");
      return;
    }
  };
}
connect();

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// ---------- keyboard ----------
const KEY_DPOS = {
  w: [+0.02, 0, 0], s: [-0.02, 0, 0],
  a: [0, -0.02, 0], d: [0, +0.02, 0],
  q: [0, 0, +0.02], e: [0, 0, -0.02],
};
const KEY_DRPY = {
  ArrowUp:    [0, +10, 0],  ArrowDown:  [0, -10, 0],
  ArrowLeft:  [+10, 0, 0],  ArrowRight: [-10, 0, 0],
  j:          [0, 0, +10],  l:          [0, 0, -10],
};
const POS_LABEL = {
  w: "x", s: "x", a: "y", d: "y", q: "z", e: "z",
};
const RPY_LABEL = {
  ArrowUp: "pitch", ArrowDown: "pitch",
  ArrowLeft: "roll", ArrowRight: "roll",
  j: "yaw", l: "yaw",
};
function describeDelta(v, scale, unit) {
  const sign = v >= 0 ? "+" : "−";
  return `${sign}${(Math.abs(v) * scale).toFixed(unit === "cm" ? 1 : 1)}${unit}`;
}

document.addEventListener("keydown", (ev) => {
  const k = ev.key;
  const scale = ev.shiftKey ? 3 : (ev.ctrlKey ? 0.2 : 1);

  // Highlight
  highlightKey(k, true);
  if (ev.shiftKey) highlightKey("Shift", true);
  if (ev.ctrlKey)  highlightKey("Control", true);

  // While IP is running, EE motion keys are locked but gripper / R / T pass
  // through so the operator can rescue a stuck episode (e.g. close gripper
  // when IP is wedging the open hand into the table).
  if (lastRunningIP) {
    if (k === "r" || k === "R" || k === "t" || k === "T") return;
    const lk2 = k.length === 1 ? k.toLowerCase() : k;
    if (KEY_DPOS[lk2] || KEY_DRPY[k]) {
      ev.preventDefault();
      return;
    }
    // Space (open) and Shift+Space (close) fall through below.
  }

  if (k === " ") {
    ev.preventDefault();
    const action = ev.shiftKey ? "close" : "open";
    send({type: "gripper", action});
    logLine(`gripper ${action}`, "grip");
    return;
  }
  if (k === "r" || k === "R") {
    if (confirm("home the robot?")) {
      send({type: "home"});
      logLine("home (joint move)", "home");
    }
    return;
  }
  if (k === "t" || k === "T") {
    send({type: "straighten"});
    logLine("straighten quat (gripper down)", "rot");
    ev.preventDefault();
    return;
  }
  const lk = k.length === 1 ? k.toLowerCase() : k;
  if (KEY_DPOS[lk]) {
    const d = KEY_DPOS[lk].map(v => v * scale);
    send({type: "ee_delta", dpos: d, drpy_deg: [0, 0, 0]});
    const axis = POS_LABEL[lk];
    const v = KEY_DPOS[lk][["x","y","z"].indexOf(axis)];
    logLine(`move ${axis} ${describeDelta(v, scale, "cm").replace("cm", " cm")}`,
            "move");
    ev.preventDefault();
    return;
  }
  if (KEY_DRPY[k]) {
    const d = KEY_DRPY[k].map(v => v * scale);
    send({type: "ee_delta", dpos: [0, 0, 0], drpy_deg: d});
    const axis = RPY_LABEL[k];
    const triple = KEY_DRPY[k];
    const idx = ["roll", "pitch", "yaw"].indexOf(axis);
    const v = triple[idx];
    logLine(`rotate ${axis} ${describeDelta(v, scale, "°").replace("°", "°")}`,
            "rot");
    ev.preventDefault();
    return;
  }
});

document.addEventListener("keyup", (ev) => {
  highlightKey(ev.key, false);
  if (!ev.shiftKey) highlightKey("Shift", false);
  if (!ev.ctrlKey)  highlightKey("Control", false);
});

// Hover keyboard map keys to send a single keystroke equivalent (so people can
// click instead of typing). Click = press once.
kbKeys.forEach(el => {
  if (el.classList.contains("kb-spacer")) return;
  el.addEventListener("click", () => {
    const k = el.dataset.k;
    if (!k) return;
    const fakeEvent = new KeyboardEvent("keydown", {
      key: k, shiftKey: false, ctrlKey: false,
    });
    document.dispatchEvent(fakeEvent);
    setTimeout(() => {
      const upEvt = new KeyboardEvent("keyup", { key: k });
      document.dispatchEvent(upEvt);
    }, 80);
  });
});

$("btn-home").onclick = () => {
  send({type: "home"});
  logLine("home (joint move)", "home");
};
$("btn-straighten").onclick = () => {
  send({type: "straighten"});
  logLine("straighten quat (button)", "rot");
};
$("btn-reset").onclick = () => {
  const promptText = $("prompt-input").value.trim() || "cube .";
  const thr = parseFloat($("gd-thresh").value);
  const t = isFinite(thr) ? thr : 0.40;
  send({type: "reset_episode", prompt: promptText,
        gd_box_threshold: t, gd_text_threshold: t});
  logLine(`reset episode, prompt="${promptText}" thresh=${t.toFixed(2)}`, "");
};
$("btn-grip-open").onclick = () => {
  send({type: "gripper", action: "open"});
  logLine("gripper open (button)", "grip");
};
$("btn-grip-close").onclick = () => {
  send({type: "gripper", action: "close"});
  logLine("gripper close (button)", "grip");
};

let lastRecording = false;
let lastRunningIP = false;
const recordBtn = $("btn-record");
recordBtn.onclick = () => {
  if (!lastRecording) {
    const name = $("demo-name").value.trim();
    send({type: "record_start", name});
  } else {
    const name = $("demo-name").value.trim();
    send({type: "record_stop", name});
  }
};
const ipBtn = $("btn-ip-run");
ipBtn.onclick = () => {
  if (!lastRunningIP) {
    if (!confirm("start IP closed-loop? robot will move autonomously")) return;
    send({type: "ip_run_start"});
  } else {
    send({type: "ip_run_stop"});
  }
};
const _origUpdateState = updateState;
updateState = (msg) => {
  lastRecording = !!msg.recording;
  lastRunningIP = !!msg.running_ip;
  _origUpdateState(msg);
};
