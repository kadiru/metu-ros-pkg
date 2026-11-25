"use strict";

const match = JSON.parse(document.getElementById("match-data").textContent);
const { frames, keys, positions } = match;
const byId = (id) => document.getElementById(id);
const canvas = byId("pitch");
const ctx = canvas.getContext("2d");
const scrub = byId("scrub");
const playButton = byId("play");
const previousButton = byId("prev");
const nextButton = byId("next");
const eventMenu = byId("events");
const teams = {
  blue: { label: "Blue", solid: "#639bff", pale: "#bcd3ff", ink: "#0b264c" },
  yellow: { label: "Yellow", solid: "#f5cd58", pale: "#ffe5a1", ink: "#382c0a" },
};
const layout = { length: 9.9, width: 6.8 };
const phaseReasons = {
  Attack: "Its team is building an attack.",
  Defend: "The opponent has possession or a pass in flight.",
  Recover: "The ball is loose.",
};
const roleTasks = {
  Keeper: "Protect its own goal and collect loose balls in reach.",
  Chaser: "Reach the loose ball and bring it under control.",
  Presser: "Close down the opposing ball carrier.",
  Carrier: "Control the ball and advance the attack.",
  Receiver: "Move into the passing lane and bring the pass under control.",
  "Left wing": "Offer an outlet on the left side of the attack.",
  "Right wing": "Offer an outlet on the right side of the attack.",
  Anchor: "Hold a deeper support position behind the ball.",
  Marker: "Track the opponent closest to this team's goal.",
  Screen: "Screen the route through the middle.",
  Cover: "Hold a deeper defensive position.",
  "Guard box": "Cover the return lane while the keeper plays the ball.",
  "Block outlet": "Cover the outlet from the opposing goal area.",
  Contain: "Drop goal-side and protect against the release.",
  Outlet: "Offer the keeper a passing option.",
  kickoff: "Drive to the assigned spot for kickoff.",
  Kickoff: "Put the centered ball back into play.",
  "Stand clear": "Keep clear of the ball during the restart.",
  Pass: "Take the awarded free kick as a pass.",
  Receive: "Move into position for the free-kick pass.",
  hold: "Hold position for the referee.",
};
const motionNames = {
  errt: "Extended RRT detour",
  brake: "Braking for clearance",
  hold: "Holding a safe waypoint",
  escape: "Clearing a blocked route",
  detour: "Collision-avoiding detour",
};

let frameIndex = match.intro.frame;
let selected = match.intro.robot;
let playing = false;
let playbackStart = 0;
let animationStart = 0;
let animationFrame = 0;
let historyStamp = "";
let canvasWidth = 0;
let canvasHeight = 0;

function robotName(index) {
  const [team, number] = keys[index];
  return `${teams[team].label} ${number}`;
}

function roleName(role) {
  if (role === "kickoff") return "Kickoff setup";
  if (role === "hold") return "Hold";
  return role;
}

function clockText(seconds) {
  const whole = Math.max(0, Math.ceil(seconds - 0.000001));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, "0")}`;
}

function replayTime(seconds) {
  const tenths = Math.round(seconds * 10);
  return `${Math.floor(tenths / 600)}:${String(Math.floor(tenths / 10) % 60).padStart(2, "0")}.${tenths % 10}`;
}

function commandName(command) {
  if (!command) return "—";
  const names = { NORMAL_START: "Play", FORCE_START: "Force start", STOP: "Stop", HALT: "Halt" };
  if (command in names) return names[command];
  const match = command.match(/^(PREPARE_KICKOFF|DIRECT_FREE|BALL_PLACEMENT)_(BLUE|YELLOW)$/);
  if (match) {
    const action = { PREPARE_KICKOFF: "kickoff setup", DIRECT_FREE: "free kick", BALL_PLACEMENT: "ball placement" };
    return `${teams[match[2].toLowerCase()].label} ${action[match[1]]}`;
  }
  return command.toLowerCase().replaceAll("_", " ");
}

const histories = keys.map((_, robot) => {
  let previous = null;
  const changes = [];
  frames.forEach((frame, tick) => {
    const role = frame.roles[robot];
    if (role === previous) return;
    changes.push({ tick, role, previous });
    previous = role;
  });
  return changes;
});

function historyIndex(robot) {
  return histories[robot].findLastIndex((entry) => entry.tick <= frameIndex);
}

function goTo(index) {
  frameIndex = Math.max(0, Math.min(index, frames.length - 1));
  render();
}

function choose(robot) {
  selected = robot;
  render();
}

const lineup = keys.map(([team, number], robot) => {
  const element = document.createElement("button");
  element.type = "button";
  const id = document.createElement("span");
  id.className = "roster-id";
  id.textContent = `${team === "blue" ? "B" : "Y"}${number}`;
  const role = document.createElement("span");
  const previous = document.createElement("span");
  previous.className = "roster-previous";
  element.append(id, role, previous);
  element.addEventListener("click", () => choose(robot));
  byId(`${team}-roster`).append(element);
  return { element, role, previous };
});

function updateLineup(frame) {
  lineup.forEach(({ element, role, previous }, robot) => {
    const current = roleName(frame.roles[robot]);
    const last = histories[robot][historyIndex(robot)].previous;
    role.textContent = current;
    previous.textContent = last === null ? "—" : roleName(last);
    element.setAttribute("aria-label", `${robotName(robot)}, ${current}, previous role ${previous.textContent}`);
    element.setAttribute("aria-pressed", String(selected === robot));
  });
}

function explain(frame, role, audit) {
  const team = teams[keys[selected][0]].label;
  if (audit?.basis === "keeper" || role === "Keeper") return "Robot 0 is the designated goalkeeper.";
  if (audit?.basis === "receiver" || role === "Receiver") return "This robot is the intended receiver. The allocator reserves it for the pass.";
  if (audit?.basis === "carrier" || role === "Carrier") return "This robot has confirmed possession and is assigned to carry the attack.";
  if (audit?.basis === "contain" || role === "Contain") return "The robots shared a grip on the ball. This field player was assigned to drop goal-side.";
  if (audit?.basis === "roster_cost") {
    const phase = frame.phases[keys[selected][0] === "blue" ? 0 : 1];
    const switchNote = audit.switch_cost > 0 && audit.allocator_previous
      ? ` Switching from ${roleName(audit.allocator_previous)} added ${audit.switch_cost.toFixed(2)} to the cost.` : "";
    return `${phaseReasons[phase] || `${team} is in ${phase.toLowerCase()}.`} The allocator picked the lowest-cost eligible lineup using travel distance and a switch penalty.${switchNote}`;
  }
  if (role === "kickoff" || role === "Kickoff") return "The referee awarded a kickoff. The restart controller set the formation and nominated a taker.";
  if (role === "hold" || role === "Stand clear" || frame.mode !== "running") return "The current referee command limits movement while the game is stopped or being restarted.";
  if (role === "Pass" || role === "Receive") return "The free-kick controller chose the passer and a receiving outlet.";
  const phase = frame.phases[keys[selected][0] === "blue" ? 0 : 1];
  return `Recorded role in ${team}’s ${phase.toLowerCase()} phase. The roster is assigned using target distance and a penalty for switching roles.`;
}

function costDetail(label, value) {
  const field = document.createElement("div");
  const dt = document.createElement("dt");
  const dd = document.createElement("dd");
  dt.textContent = label;
  dd.textContent = value;
  field.append(dt, dd);
  return field;
}

function updateHistory() {
  const current = historyIndex(selected);
  previousButton.disabled = current <= 0;
  nextButton.disabled = current + 1 >= histories[selected].length;
  const stamp = `${selected}/${current}`;
  if (stamp === historyStamp) return;
  historyStamp = stamp;
  const list = byId("history");
  list.replaceChildren();
  for (const change of histories[selected].slice(Math.max(0, current - 3), current + 1).reverse()) {
    const li = document.createElement("li");
    const button = document.createElement("button");
    button.type = "button";
    const name = document.createElement("span");
    name.textContent = roleName(change.role);
    const time = document.createElement("time");
    time.textContent = replayTime(frames[change.tick].t);
    button.append(name, time);
    button.addEventListener("click", () => { pause(); goTo(change.tick); });
    li.append(button);
    list.append(li);
  }
}

function updateInspector(frame) {
  const role = frame.roles[selected];
  const audit = frame.allocation?.[selected];
  const command = audit?.command;
  const last = histories[selected][historyIndex(selected)].previous;
  const jersey = byId("jersey");
  jersey.className = `jersey ${keys[selected][0]}`;
  jersey.textContent = robotName(selected);
  byId("role").textContent = roleName(role);
  byId("previous").textContent = last === null ? "—" : roleName(last);
  byId("reason").textContent = explain(frame, role, audit);
  let task = audit?.task || roleTasks[role] || roleName(role);
  if (audit && !command) task = "Hold for the current referee instruction.";
  else if (audit?.pass_to != null) task = `Pass to ${robotName(audit.pass_to)}.`;
  else if (command?.[4] > 0 && audit?.task === "Line up a shot") task = "Shoot at goal.";
  const motion = frame.motion?.[selected];
  if (command && motion in motionNames) task += ` ${motionNames[motion]}.`;
  byId("task").textContent = task;
  const target = byId("target");
  target.hidden = !audit?.target || !command || role === "hold" || role === "Stand clear";
  if (!target.hidden) target.textContent = `Role target ${audit.target.map((v) => v.toFixed(2)).join(", ")} m`;
  const costs = byId("costs");
  costs.hidden = audit?.distance_m == null;
  if (!costs.hidden) {
    costs.replaceChildren(
      costDetail("Travel", `${audit.distance_m.toFixed(2)} m`),
      costDetail("Role switch", audit.switch_cost.toFixed(2)),
      costDetail("Lineup cost", audit.roster_cost.toFixed(2)),
    );
  }
  const commandDetail = byId("command-details");
  commandDetail.hidden = !command;
  if (command) {
    const [forward, lateral, turn, roller, kick] = command;
    byId("command").textContent = `Forward ${forward.toFixed(2)} m/s · Lateral ${lateral.toFixed(2)} m/s · Turn ${turn.toFixed(2)} rad/s · Roller ${roller > 0 ? "on" : "off"}${kick > 0 ? ` · Kick ${kick.toFixed(2)} m/s` : ""}`;
  }
  updateHistory();
}

function render() {
  const frame = frames[frameIndex];
  byId("blue-score").textContent = frame.score[0];
  byId("yellow-score").textContent = frame.score[1];
  byId("clock").textContent = frame.left === null ? "—" : clockText(frame.left);
  byId("period").textContent = frame.stage || "Practice";
  byId("referee").textContent = commandName(frame.command);
  const receiver = frame.roles.indexOf("Receiver");
  byId("owner").textContent = receiver >= 0 ? `Pass to ${robotName(receiver)}`
    : frame.owner === null ? "Loose ball" : robotName(frame.owner);
  byId("blue-phase").textContent = `Blue · ${frame.phases[0]}`;
  byId("yellow-phase").textContent = `Yellow · ${frame.phases[1]}`;
  byId("replay-time").textContent = `${replayTime(frame.t)} / ${replayTime(frames.at(-1).t)}`;
  scrub.value = frameIndex;
  scrub.setAttribute("aria-valuetext", `${replayTime(frame.t)} elapsed`);
  updateInspector(frame);
  updateLineup(frame);
  drawField();
}

function scale() {
  return Math.min(canvasWidth / layout.length, canvasHeight / layout.width);
}

function project(x, y) {
  const k = scale();
  return [canvasWidth / 2 + x * k, canvasHeight / 2 - y * k];
}

function traceCircle(x, y, radius, fill, stroke, width = 1) {
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke(); }
}

function drawField() {
  if (!canvasWidth || !canvasHeight) return;
  const k = scale();
  const frame = frames[frameIndex];
  const points = positions[frameIndex];
  ctx.fillStyle = "#16382e";
  ctx.fillRect(0, 0, canvasWidth, canvasHeight);
  const [left, top] = project(-4.5, 3);
  const [right, bottom] = project(4.5, -3);
  const gradient = ctx.createLinearGradient(0, top, 0, bottom);
  gradient.addColorStop(0, "#245041");
  gradient.addColorStop(1, "#214b3e");
  ctx.fillStyle = gradient;
  ctx.fillRect(left, top, right - left, bottom - top);
  for (let band = 1; band < 9; band += 2) {
    ctx.fillStyle = "#ffffff08";
    ctx.fillRect(left + band * k, top, k, bottom - top);
  }
  ctx.strokeStyle = "#bad6cbaa";
  ctx.lineWidth = 1.25;
  ctx.strokeRect(left, top, right - left, bottom - top);
  const [cx, cy] = project(0, 0);
  ctx.beginPath();
  ctx.moveTo(cx, top);
  ctx.lineTo(cx, bottom);
  ctx.stroke();
  traceCircle(cx, cy, .5 * k, null, "#bad6cbaa");
  traceCircle(cx, cy, 2, "#bad6cb");
  for (const sign of [-1, 1]) {
    const [x, y] = project(sign < 0 ? -4.5 : 3.5, 1);
    ctx.strokeStyle = "#bad6cbaa";
    ctx.strokeRect(x, y, k, 2 * k);
    const goalTeam = sign === -frame.sign ? "blue" : "yellow";
    const [gx, gy] = project(sign * 4.5, .5);
    ctx.strokeStyle = teams[goalTeam].solid;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(gx, gy);
    ctx.lineTo(gx + sign * .21 * k, gy);
    ctx.lineTo(gx + sign * .21 * k, gy + k);
    ctx.lineTo(gx, gy + k);
    ctx.stroke();
  }
  const target = frame.allocation?.[selected]?.target;
  if (target && frame.allocation[selected].command && frame.roles[selected] !== "Stand clear") {
    const [tx, ty] = project(...target);
    const [rx, ry] = project(points[2 + 3 * selected], points[3 + 3 * selected]);
    if (Math.hypot(tx - rx, ty - ry) > 12) {
      ctx.strokeStyle = teams[keys[selected][0]].pale;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 5]);
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.setLineDash([]);
      traceCircle(tx, ty, 4, null, teams[keys[selected][0]].pale, 1.4);
    }
  }
  keys.forEach(([team, number], robot) => {
    const offset = 2 + 3 * robot;
    const [x, y] = project(points[offset], points[offset + 1]);
    const yaw = points[offset + 2];
    const style = teams[team];
    const radius = Math.max(7, Math.min(13, .15 * k));
    if (robot === selected) traceCircle(x, y, radius + 4, null, style.pale, 2);
    traceCircle(x, y, radius, style.solid, style.ink, 1);
    const dx = Math.cos(yaw), dy = -Math.sin(yaw);
    ctx.strokeStyle = "#f6faff";
    ctx.lineWidth = 2.3;
    ctx.beginPath();
    ctx.moveTo(x + dx * (radius - 4), y + dy * (radius - 4));
    ctx.lineTo(x + dx * (radius + 2), y + dy * (radius + 2));
    ctx.stroke();
    ctx.fillStyle = style.ink;
    ctx.font = `700 ${Math.max(10, Math.round(radius))}px system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(number), x - dx * 1.5, y - dy * 1.5);
  });
  const [bx, by] = project(points[0], points[1]);
  traceCircle(bx, by, Math.max(3.4, .058 * k), "#ffa95f", "#492a15", 1);
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvasWidth = rect.width;
  canvasHeight = rect.height;
  const pixelRatio = window.devicePixelRatio || 1;
  canvas.width = Math.round(canvasWidth * pixelRatio);
  canvas.height = Math.round(canvasHeight * pixelRatio);
  ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  drawField();
}

canvas.addEventListener("click", (event) => {
  const rect = canvas.getBoundingClientRect();
  let closest = null;
  let closestDistance = 20;
  keys.forEach((_, index) => {
    const offset = 2 + 3 * index;
    const [x, y] = project(positions[frameIndex][offset], positions[frameIndex][offset + 1]);
    const distance = Math.hypot(event.clientX - rect.left - x, event.clientY - rect.top - y);
    if (distance < closestDistance) { closestDistance = distance; closest = index; }
  });
  if (closest !== null) choose(closest);
});

function pause() {
  playing = false;
  window.cancelAnimationFrame(animationFrame);
  playButton.textContent = "Play";
}

function animate(now) {
  if (!playing) return;
  const realTime = playbackStart + (now - animationStart) / 1000;
  let next = frameIndex;
  while (next + 1 < frames.length && frames[next + 1].t <= realTime) next += 1;
  if (next !== frameIndex) goTo(next);
  if (next >= frames.length - 1) pause();
  else animationFrame = window.requestAnimationFrame(animate);
}

playButton.addEventListener("click", () => {
  if (playing) { pause(); return; }
  if (frameIndex >= frames.length - 1) goTo(0);
  playing = true;
  playbackStart = frames[frameIndex].t;
  animationStart = performance.now();
  playButton.textContent = "Pause";
  animationFrame = window.requestAnimationFrame(animate);
});

scrub.max = frames.length - 1;
scrub.addEventListener("input", () => { pause(); goTo(Number(scrub.value)); });
previousButton.addEventListener("click", () => {
  pause();
  const index = historyIndex(selected);
  if (index > 0) goTo(histories[selected][index - 1].tick);
});
nextButton.addEventListener("click", () => {
  pause();
  const next = histories[selected][historyIndex(selected) + 1];
  if (next) goTo(next.tick);
});
for (const [tick, label] of match.events) {
  const option = document.createElement("option");
  option.value = tick;
  option.textContent = `${replayTime(frames[tick].t)} · ${label}`;
  eventMenu.append(option);
}
eventMenu.addEventListener("change", () => {
  if (eventMenu.value === "") return;
  pause();
  goTo(Number(eventMenu.value));
  eventMenu.value = "";
});
new ResizeObserver(resizeCanvas).observe(canvas);
render();
