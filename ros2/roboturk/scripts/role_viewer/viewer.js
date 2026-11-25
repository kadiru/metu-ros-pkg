"use strict";

const match = JSON.parse(document.getElementById("match-data").textContent);
const { frames = [], keys = [], positions = [], events = [] } = match;
const livePage = match.live === true;
const ownTeam = match.session?.team;
const roleWindow = 10;
const byId = (id) => document.getElementById(id);
const canvas = byId("pitch");
const ctx = canvas.getContext("2d");
const scrub = byId("scrub");
const playButton = byId("play");
const previousButton = byId("prev");
const nextButton = byId("next");
const liveButton = byId("go-live");
const eventScroll = byId("event-scroll");
const eventTrack = byId("event-track");
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
const roleColors = {
  Keeper: "#b4b6f1", Chaser: "#84dec5", Presser: "#ffae94",
  Carrier: "#ace38c", Receiver: "#86d4fa", "Left wing": "#8cb0ff", "Right wing": "#b7a3f4",
  Anchor: "#c1d69f", Marker: "#efbf87", Screen: "#ebd18f", Cover: "#edb1a0",
  "Guard box": "#d0a9cf", "Block outlet": "#d9b29b", Contain: "#f8a19e", Outlet: "#97dfb2",
  kickoff: "#b8c7d5", Kickoff: "#9ec5e9", "Stand clear": "#9daebc",
  Pass: "#9fcce6", Receive: "#95dcea", hold: "#a0afc4",
};
const eventColors = {
  goal: "#b2e58b", shot: "#ffbd89", pass: "#9dabee", chip: "#c9b2ff",
  kickoff: "#90d0b6", restart: "#eac575", period: "#b7bfcf",
};

let frameIndex = match.intro?.frame ?? 0;
let selected = match.intro?.robot ?? 0;
let initialLimits;
let limits;
let slidersEditable;
let lastSliderValues;
let limitWrites = Promise.resolve();
const pendingLimits = new Map();
const limitsByFrame = [];
let following = livePage;
let finished = false;
let matchFailed = false;
let feedError = false;
let solo = false;
let eventWindowStart = null;
let eventStamp = "";
let playing = false;
let playbackStart = 0;
let animationStart = 0;
let animationFrame = 0;
let historyStamp = "";
let canvasWidth = 0;
let canvasHeight = 0;
const teamEnabled = { blue: true, yellow: true };
const robotEnabled = [];
const overlayEnabled = new Set(match.intro?.overlays ?? []);
const histories = [];
const lineup = [];
const filterButtons = [];
const teamButtons = {};
const motionControls = createLimitControls(byId("motion-limits"), match.limit_ranges, changeLimit);

function robotName(index) {
  const [team, number] = keys[index];
  return `${team[0].toUpperCase()}${number}`;
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

function addRoleSample(frame, tick) {
  frame.roles.forEach((role, robot) => {
    const history = histories[robot];
    const previous = history.at(-1)?.role ?? null;
    if (role !== previous) history.push({ tick, role, previous });
  });
  limitsByFrame.push(frame.limits || limitsByFrame.at(-1) || initialLimits);
}

function historyIndex(robot) {
  return histories[robot].findLastIndex((entry) => entry.tick <= frameIndex);
}

function goTo(index) {
  frameIndex = Math.max(0, Math.min(index, frames.length - 1));
  render();
}

function seek(index) {
  following = false;
  pause();
  goTo(index);
}

function choose(robot) {
  selected = robot;
  render();
}

function makeRosterButton(team, number, robot) {
  const element = document.createElement("button");
  element.type = "button";
  const id = document.createElement("span");
  id.className = "roster-id";
  id.textContent = `${team === "blue" ? "B" : "Y"}${number}`;
  const role = document.createElement("span");
  const previous = document.createElement("span");
  previous.className = "roster-previous";
  element.append(id, role, previous);
  element.addEventListener("click", () => {
    if (overlayEnabled.has(robot)) overlayEnabled.delete(robot);
    else overlayEnabled.add(robot);
    choose(robot);
  });
  byId(`${team}-roster`).append(element);
  return { element, role, previous };
}

function filterButton(parent, label, name, callback) {
  const button = document.createElement("button");
  button.type = "button";
  button.textContent = label;
  button.title = name;
  button.setAttribute("aria-label", name);
  button.addEventListener("click", callback);
  parent.append(button);
  return button;
}

function initializeTeams() {
  const groups = {};
  for (const [team, { label }] of Object.entries(teams)) {
    const group = document.createElement("div");
    group.className = `filter-group ${team}`;
    groups[team] = group;
    byId(`${team}-overlay`).addEventListener("click", () => {
      const roster = keys.flatMap(([color], robot) => color === team ? [robot] : []);
      const all = roster.every((robot) => overlayEnabled.has(robot));
      roster.forEach((robot) => {
        if (all) overlayEnabled.delete(robot);
        else overlayEnabled.add(robot);
      });
      render();
    });
    byId("field-filters").append(group);
    teamButtons[team] = filterButton(group, label, `Show ${label} team`, () => {
      exitSoloForFilter();
      teamEnabled[team] = !teamVisible(team);
      if (teamEnabled[team] && !keys.some(([color], robot) => color === team && robotEnabled[robot])) {
        keys.forEach(([color], robot) => { if (color === team) robotEnabled[robot] = true; });
      }
      render();
    });
  }
  keys.forEach(([team, number], robot) => {
    histories.push([]);
    robotEnabled.push(true);
    lineup.push(makeRosterButton(team, number, robot));
    filterButtons.push(filterButton(groups[team], robotName(robot), `Show ${robotName(robot)}`, () => {
      exitSoloForFilter();
      if (!teamEnabled[team]) {
        keys.forEach(([color], index) => { if (color === team) robotEnabled[index] = false; });
        teamEnabled[team] = true;
      }
      robotEnabled[robot] = !robotEnabled[robot];
      render();
    }));
  });
}

function isVisible(robot) {
  if (solo) return robot === selected;
  return teamEnabled[keys[robot][0]] && robotEnabled[robot];
}

function teamVisible(team) {
  return keys.some(([color], robot) => color === team && isVisible(robot));
}

function exitSoloForFilter() {
  if (!solo) return;
  robotEnabled.forEach((_, robot) => { robotEnabled[robot] = robot === selected; });
  teamEnabled.blue = teamEnabled.yellow = true;
  solo = false;
}

function updateFilters() {
  for (const [team, button] of Object.entries(teamButtons)) {
    button.setAttribute("aria-pressed", String(teamVisible(team)));
  }
  filterButtons.forEach((button, robot) => button.setAttribute("aria-pressed", String(isVisible(robot))));
  byId("solo").setAttribute("aria-pressed", String(solo));
  byId("solo").disabled = !frames.length;
  byId("show-all").disabled = !frames.length || keys.every((_, robot) => isVisible(robot));
}

function updateLineup(frame) {
  for (const team of Object.keys(teams)) {
    const button = byId(`${team}-overlay`);
    const roster = keys.flatMap(([color], robot) => color === team ? [robot] : []);
    const enabled = roster.filter((robot) => overlayEnabled.has(robot)).length;
    button.disabled = !roster.length;
    button.setAttribute("aria-pressed", enabled === 0 ? "false" : enabled === roster.length ? "true" : "mixed");
  }
  lineup.forEach(({ element, role, previous }, robot) => {
    const current = roleName(frame.roles[robot]);
    const last = histories[robot][historyIndex(robot)].previous;
    role.textContent = current;
    previous.textContent = last === null ? "—" : roleName(last);
    element.setAttribute("aria-label", `${robotName(robot)}, ${current}, previous role ${previous.textContent}`);
    element.setAttribute("aria-pressed", String(overlayEnabled.has(robot)));
    if (selected === robot) element.setAttribute("aria-current", "true");
    else element.removeAttribute("aria-current");
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

function updateHistory(now) {
  const history = histories[selected];
  const current = historyIndex(selected);
  previousButton.disabled = current <= 0;
  nextButton.disabled = current + 1 >= history.length;
  const stamp = `${selected}/${current}/${Math.floor(now * 5)}`;
  if (stamp === historyStamp) return;
  historyStamp = stamp;
  const start = now - roleWindow;
  const track = byId("role-track");
  const first = Math.max(0, history.findLastIndex(({ tick }, index) =>
    index <= current && frames[tick].t <= start));
  const blank = document.createElement("span");
  blank.style.flexBasis = `${Math.max(0, frames[history[first].tick].t - start) / roleWindow * 100}%`;
  track.replaceChildren(blank);
  for (let index = first; index <= current; index += 1) {
    const { tick, role } = history[index];
    const begin = Math.max(start, frames[tick].t);
    const end = index === current ? now : frames[history[index + 1].tick].t;
    const fraction = Math.max(.001, (end - begin) / roleWindow);
    const chip = document.createElement("button");
    chip.type = "button";
    chip.className = "role-segment";
    chip.style.flex = `0 0 ${100 * fraction}%`;
    chip.style.setProperty("--role-color", roleColor(role));
    chip.title = `${roleName(role)} · ${replayTime(frames[tick].t)}`;
    chip.setAttribute("aria-label", chip.title);
    if (fraction > .10) chip.textContent = roleName(role);
    chip.addEventListener("click", () => seek(tick));
    track.append(chip);
  }
  const elapsed = now - frames[history[current].tick].t;
  const changeCount = history.slice(first, current + 1)
    .filter(({ tick, previous }) => previous !== null && frames[tick].t >= start).length;
  byId("role-duration").textContent = `${roleName(history[current].role)} for ${elapsed.toFixed(1)} s · ${changeCount} role ${changeCount === 1 ? "change" : "changes"}`;
  const recent = history.slice(Math.max(first, current - 2), current + 1).reverse();
  byId("role-changes").replaceChildren(...recent.map(({ tick, role }) => {
    const button = document.createElement("button");
    button.type = "button";
    button.style.setProperty("--role-color", roleColor(role));
    const time = document.createElement("time");
    time.textContent = replayTime(frames[tick].t);
    const label = document.createElement("strong");
    label.textContent = roleName(role);
    const task = document.createElement("span");
    const recorded = frames[tick].allocation?.[selected];
    task.textContent = recorded?.task || roleTasks[role] || roleName(role);
    button.append(time, label, task);
    button.addEventListener("click", () => seek(tick));
    return button;
  }));
}

function roleColor(role) {
  return roleColors[role] || "#a6b5c5";
}

function initialLimitValues(values) {
  if (Array.isArray(values)) {
    // Older live recordings carried only translation and rotation.
    const [max_speed, max_omega] = values;
    return { blue: { max_speed, max_omega }, yellow: { max_speed, max_omega } };
  }
  return values;
}

function changeLimit(name, value) {
  const feedback = byId("limit-feedback");
  feedback.hidden = true;
  pendingLimits.set(name, value);
  limitWrites = limitWrites.then(async () => {
    try {
      const response = await fetch("limits", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ [name]: value }),
      });
      if (!response.ok) throw new Error((await response.json()).error);
    } catch (error) {
      if (pendingLimits.get(name) === value) pendingLimits.delete(name);
      feedback.textContent = error.message || "Could not save the limit.";
      feedback.hidden = false;
      lastSliderValues = null;
      updateLimitControls();
    }
  });
}

function updateLimitControls() {
  limits = limitsByFrame[frameIndex] || initialLimits;
  const editable = Boolean(limits && frames.length && match.edit_limits && following
    && !finished && !matchFailed && !feedError);
  if (limits === lastSliderValues && editable === slidersEditable) return;
  lastSliderValues = limits;
  slidersEditable = editable;
  for (const [name, value] of pendingLimits) {
    if (limits?.blue[name] === value) pendingLimits.delete(name);
  }
  motionControls.show(limits, editable, editable ? pendingLimits : undefined);
}

function updateGauges(audit) {
  const values = audit?.command ?? (audit ? [0, 0, 0, 0, 0] : null);
  byId("command-details").hidden = !values;
  byId("command-unavailable").hidden = !!values;
  if (!values) {
    byId("command-unavailable").textContent = "Commands are unavailable for this recording.";
    return;
  }
  const [forward, lateral, turn, roller, kick] = values;
  const { max_speed: maxSpeed, max_omega: maxTurn } = limits[keys[selected][0]];
  const speed = Math.hypot(forward, lateral);
  const direction = Math.atan2(-forward, -lateral);
  const needle = byId("vector-needle");
  needle.hidden = speed < .01;
  needle.style.setProperty("--length", `${29 * Math.min(speed / maxSpeed, 1)}px`);
  needle.style.setProperty("--angle", `${direction}rad`);
  byId("speed").textContent = `${speed.toFixed(2)} m/s`;
  byId("speed-scale").textContent = `Ring at ${maxSpeed} m/s`;
  const measured = byId("measured-speed");
  measured.hidden = audit.measured_speed == null;
  if (!measured.hidden) measured.textContent = `Measured ${audit.measured_speed.toFixed(2)} m/s`;
  byId("translation").setAttribute("aria-label",
    `Commanded forward velocity ${forward.toFixed(2)} and leftward velocity ${lateral.toFixed(2)} meters per second`);
  const turnLabel = `${turn > .005 ? "Left " : turn < -.005 ? "Right " : ""}${Math.abs(turn).toFixed(2)} rad/s`;
  byId("turn").textContent = turnLabel;
  byId("turn-left").textContent = `↶ ${maxTurn}`;
  byId("turn-right").textContent = `${maxTurn} ↷`;
  byId("rotation").setAttribute("aria-label", `Commanded rotation ${turnLabel}`);
  const shift = -50 * Math.max(-1, Math.min(turn / maxTurn, 1));
  byId("yaw-fill").style.setProperty("--start", `${50 + Math.min(0, shift)}%`);
  byId("yaw-fill").style.setProperty("--size", `${Math.abs(shift)}%`);
  byId("ball-command").textContent = `Roller ${roller > 0 ? "on" : "off"}${kick > 0 ? ` · Kick ${kick.toFixed(2)} m/s` : ""}`;
}

function updateEventTrack(now) {
  const end = livePage ? now : Math.min(frames.at(-1)?.t ?? 0, Math.max(60, now));
  const stamp = `${Math.floor(end * 5)}/${events.length}/${eventScroll.clientWidth}`;
  if (stamp === eventStamp) {
    eventTrack.querySelector("[aria-current]")?.removeAttribute("aria-current");
    eventTrack.querySelector(`[data-tick="${frameIndex}"]`)?.setAttribute("aria-current", "true");
    return;
  }
  eventStamp = stamp;
  const start = Math.max(0, end - 60);
  const pixelsPerSecond = 28;
  const wasAtRight = eventWindowStart === null ? livePage
    : eventTrack.offsetWidth - eventScroll.clientWidth - eventScroll.scrollLeft < 12;
  const previousAnchor = eventScroll.scrollLeft + ((eventWindowStart ?? start) - start) * pixelsPerSecond;
  eventWindowStart = start;
  const nodes = document.createDocumentFragment();
  const xAt = (time) => 8 + (time - start) * pixelsPerSecond;
  for (let mark = Math.ceil(start / 10) * 10; mark <= end; mark += 10) {
    const line = document.createElement("div");
    line.className = "event-gridline";
    line.style.left = `${xAt(mark)}px`;
    const label = document.createElement("time");
    label.textContent = replayTime(mark);
    line.append(label);
    nodes.append(line);
  }
  const lanes = [];
  const visible = events.filter(([tick, , kind]) => kind !== "kick_trace" && frames[tick] && start <= frames[tick].t && frames[tick].t <= end);
  for (const [tick, label, kind, team] of visible) {
    const time = replayTime(frames[tick].t);
    const left = xAt(frames[tick].t);
    const free = lanes.findIndex((end) => end <= left);
    const lane = free < 0 ? lanes.length : free;
    const compact = label.replace(/\b(Blue|Yellow) (\d+)\b/g, (_, color, id) => color[0] + id);
    const caption = kind === "pass" ? compact.replace(" passes to ", " → ") : compact;
    lanes[lane] = left + Math.min(164, 54 + caption.length * 6) + 6;
    const chip = document.createElement("button");
    chip.type = "button";
    chip.className = "event-chip";
    chip.title = `${time} · ${compact}`;
    chip.setAttribute("aria-label", chip.title);
    chip.dataset.tick = tick;
    if (tick === frameIndex) chip.setAttribute("aria-current", "true");
    chip.style.left = `${left}px`;
    chip.style.top = `${7 + lane * 30}px`;
    chip.style.setProperty("--event-color",
      kind === "goal" || kind === "kickoff" ? teams[team]?.pale || eventColors[kind]
        : eventColors[kind] || eventColors.period);
    const clock = document.createElement("time");
    clock.textContent = time;
    chip.append(clock, `  ${caption}`);
    chip.addEventListener("click", () => seek(tick));
    nodes.append(chip);
  }
  if (!visible.length) {
    const empty = document.createElement("span");
    empty.className = "event-empty";
    empty.textContent = frames.length ? "No match events in this window." : "Waiting for the first match event.";
    nodes.append(empty);
  }
  eventTrack.style.width = `${Math.max(eventScroll.clientWidth, xAt(end) + 172)}px`;
  eventTrack.style.height = `${Math.max(72, 30 * lanes.length + 25)}px`;
  eventTrack.replaceChildren(nodes);
  eventScroll.scrollLeft = wasAtRight ? eventTrack.offsetWidth - eventScroll.clientWidth : previousAnchor;
  updateEventNavigation();
}

function updateEventNavigation() {
  byId("earlier-events").disabled = eventScroll.scrollLeft <= 1;
  byId("later-events").disabled =
    eventScroll.scrollLeft >= eventScroll.scrollWidth - eventScroll.clientWidth - 1;
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
  byId("current-card").style.setProperty("--role-color", roleColor(role));
  byId("prior-card").style.setProperty("--role-color", roleColor(last));
  byId("reason").textContent = explain(frame, role, audit);
  const play = frame.plays?.[keys[selected][0] === "blue" ? 0 : 1];
  byId("play-section").hidden = !play;
  if (play) {
    byId("plan-strategy").textContent = play.strategy;
    byId("plan-passing").textContent = play.passing;
    byId("plan-play").textContent = play.play;
    byId("plan-step").textContent = play.step;
    byId("plan-reason").textContent = play.reason;
  }
  let task = audit?.task || roleTasks[role] || roleName(role);
  if (audit && !command) task = "Hold for the current referee instruction.";
  else if (audit?.pass_to != null) task = `Pass to ${robotName(audit.pass_to)}.`;
  else if (audit?.kick_elevation > 0) task = "Chip to the receiver.";
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
  updateGauges(audit);
  updateHistory(frame.t);
}

function render() {
  updateFeed();
  updateFilters();
  updateLimitControls();
  const frame = frames[frameIndex];
  playButton.disabled = !frame || (livePage && following && !finished);
  scrub.disabled = !frame;
  previousButton.disabled = nextButton.disabled = !frame;
  if (!frame) {
    updateEventTrack(0);
    drawField();
    return;
  }
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
  byId("blue-strategy-label").textContent = frame.plays?.[0]?.strategy || "Current / previous";
  byId("yellow-strategy-label").textContent = frame.plays?.[1]?.strategy || "Current / previous";
  byId("replay-time").textContent = `${replayTime(frame.t)} / ${replayTime(frames.at(-1).t)}`;
  scrub.max = frames.length - 1;
  scrub.value = frameIndex;
  scrub.setAttribute("aria-valuetext", `${replayTime(frame.t)} elapsed`);
  updateInspector(frame);
  updateLineup(frame);
  updateEventTrack(frame.t);
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

function strokePath(points, color, width, dash = []) {
  if (points.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(...points[0]);
  for (const point of points.slice(1)) ctx.lineTo(...point);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawOverlays(frame, points) {
  const shots = new Map();
  const results = new Map();
  for (const [tick, , kind, team, details] of events) {
    if (!details || tick > frameIndex || frames[tick]?.t < frame.t - 10) continue;
    const robot = details.actor;
    if (!overlayEnabled.has(robot) || !isVisible(robot)) continue;
    if (kind === "kick_trace") results.set(`${robot}:${details.kick_tick}`, details);
    else if (details.origin) {
      const lane = `${robot}:${kind === "chip" ? "chip" : "flat"}`;
      if (kind === "shot" || kind === "chip" || shots.get(lane)?.kind !== "shot") {
        shots.set(lane, { tick, kind, team, ...details });
      }
    }
  }
  for (const robot of overlayEnabled) {
    const audit = frame.allocation?.[robot];
    if (!isVisible(robot) || !audit?.target || !audit.command || frame.roles[robot] === "Stand clear") continue;
    const offset = 2 + 3 * robot;
    const origin = project(points[offset], points[offset + 1]);
    const target = project(...audit.target);
    if (Math.hypot(target[0] - origin[0], target[1] - origin[1]) <= 12) continue;
    const color = teams[keys[robot][0]].pale;
    strokePath([origin, target], `${color}a0`, 1.3, [5, 5]);
    traceCircle(...target, 4, null, color, 1.4);
  }
  const descriptions = [];
  for (const launch of shots.values()) {
    const result = results.get(`${launch.actor}:${launch.tick}`);
    const end = Math.min(result?.tick ?? frameIndex, launch.tick + Math.ceil(2 / (match.dt || .04)));
    const track = [launch.origin];
    for (let i = launch.tick + 1; i <= end; i += 1) {
      track.push([positions[i][0], positions[i][1], frames[i].ball_z ?? 0]);
    }
    if (result) track.push(result.point);
    const path = track.map(([x, y]) => project(x, y));
    const color = teams[launch.team].pale;
    const lofted = launch.elevation > 0;
    if (lofted) {
      strokePath(path, "#101b1aaa", 2, [2, 4]);
      strokePath(track.map(([x, y, z]) => {
        const [px, py] = project(x, y);
        return [px, py - Math.max(0, z - 0.022) * scale() * 0.65];
      }), color, 2.1, [2, 4]);
    } else {
      strokePath(path, "#14201b", 4);
      strokePath(path, color, 2);
    }
    traceCircle(...path[0], 3, null, color, 1.4);
    const goal = result?.outcome === "goal mouth";
    if (goal) {
      const [x, y] = path.at(-1);
      traceCircle(x, y, 6, "#13241d", color, 1.8);
      traceCircle(x, y, 2.2, color);
      ctx.font = "700 11px system-ui";
      ctx.textAlign = result.point[0] > 0 ? "right" : "left";
      ctx.textBaseline = "bottom";
      const dx = result.point[0] > 0 ? -10 : 10;
      ctx.lineWidth = 3;
      ctx.strokeStyle = "#13241d";
      ctx.strokeText(`${result.speed.toFixed(2)} m/s`, x + dx, y - 7);
      ctx.fillStyle = color;
      ctx.fillText(`${result.speed.toFixed(2)} m/s`, x + dx, y - 7);
    }
    let detail = `${robotName(launch.actor)} ${lofted ? "chip" : launch.kind === "shot" ? "shot" : "kick"} · ${launch.speed.toFixed(1)} m/s launch`;
    if (goal || result?.outcome === "wide") detail += ` · ${result.speed.toFixed(2)} m/s at goal line · ${Math.abs(result.point[1]).toFixed(2)} m from center · ${result.point[2].toFixed(2)} m high${goal ? "" : " · wide"}`;
    else if (lofted && result) detail += ` · ${result.peak_z.toFixed(2)} m peak`;
    descriptions.push(detail);
  }
  const readout = byId("kick-readout");
  const text = descriptions.join("  |  ");
  if (readout.textContent !== text) readout.textContent = text;
  readout.hidden = !text;
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
    const goalTeam = sign === -(frame?.sign ?? 1) ? "blue" : "yellow";
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
  if (!frame) return;
  drawOverlays(frame, points);
  keys.forEach(([team], robot) => {
    if (!isVisible(robot)) return;
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
    ctx.font = `700 ${Math.max(8, Math.round(.81 * radius))}px system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(robotName(robot), x - dx * 1.5, y - dy * 1.5);
  });
  const [bx, by] = project(points[0], points[1]);
  const lift = Math.max(0, (frame.ball_z ?? 0) - .022);
  if (lift > .02) traceCircle(bx, by, 3, "#0e1f22aa");
  traceCircle(bx, by - lift * k * .65, Math.max(3.4, .058 * k), "#ffa95f", "#492a15", 1);
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
  if (!frames.length) return;
  const rect = canvas.getBoundingClientRect();
  let closest = null;
  let closestDistance = 20;
  keys.forEach((_, index) => {
    if (!isVisible(index)) return;
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
  if (!frames.length) return;
  if (playing) { pause(); return; }
  if (frameIndex >= frames.length - 1) goTo(0);
  playing = true;
  playbackStart = frames[frameIndex].t;
  animationStart = performance.now();
  playButton.textContent = "Pause";
  animationFrame = window.requestAnimationFrame(animate);
});

scrub.addEventListener("input", () => seek(Number(scrub.value)));
previousButton.addEventListener("click", () => {
  const index = historyIndex(selected);
  if (index > 0) seek(histories[selected][index - 1].tick);
});
nextButton.addEventListener("click", () => {
  const next = histories[selected][historyIndex(selected) + 1];
  if (next) seek(next.tick);
});
liveButton.addEventListener("click", () => {
  pause();
  following = true;
  eventWindowStart = null;
  eventStamp = "";
  goTo(frames.length - 1);
});
byId("solo").addEventListener("click", () => { solo = !solo; render(); });
byId("show-all").addEventListener("click", () => {
  solo = false;
  teamEnabled.blue = teamEnabled.yellow = true;
  robotEnabled.fill(true);
  render();
});
for (const [id, direction] of [["earlier-events", -1], ["later-events", 1]]) {
  byId(id).addEventListener("click", () => eventScroll.scrollBy(direction * eventScroll.clientWidth * .75, 0));
}
eventScroll.addEventListener("scroll", updateEventNavigation, { passive: true });

function updateFeed() {
  if (!livePage) return;
  let [state, label] = ["waiting", "Waiting for Webots"];
  if (matchFailed) [state, label] = ["offline", "Match failed to run"];
  else if (finished) [state, label] = ["replay", "Match finished"];
  else if (feedError) [state, label] = ["offline", "Reconnecting"];
  else if (frames.length) [state, label] = following ? ["live", "Live · Webots"] : ["review", "Reviewing live match"];
  byId("feed-state").dataset.state = state;
  byId("feed-label").textContent = label;
  liveButton.hidden = !frames.length || (finished && frameIndex === frames.length - 1);
  liveButton.disabled = following && !finished;
  liveButton.textContent = finished ? "Jump to end" : following ? "Following live" : "Go live";
}

async function poll(offset = 0) {
  try {
    const response = await fetch(`telemetry?offset=${offset}`);
    if (!response.ok) throw new Error("Telemetry request failed");
    matchFailed = response.headers.has("X-Match-Failed");
    const batch = await response.text();
    for (const line of batch.trimEnd().split("\n")) {
      if (!line) continue;
      const item = JSON.parse(line);
      if (item.header) {
        keys.push(...item.header.keys);
        initialLimits = initialLimitValues(item.header.limits);
        if (ownTeam) {
          const ownRobot = keys.findIndex(([team, number]) => team === ownTeam && number === 1);
          if (ownRobot >= 0) selected = ownRobot;
        }
        initializeTeams();
      } else if (item.done) {
        finished = true;
      } else {
        addRoleSample(item.frame, frames.length);
        frames.push(item.frame);
        positions.push(item.position);
        events.push(...item.events);
      }
    }
    offset = Number(response.headers.get("X-Telemetry-Offset"));
    feedError = false;
    if (following && frames.length) frameIndex = frames.length - 1;
    if (batch || !frames.length) render();
    else {
      updateFeed();
      updateLimitControls();
    }
  } catch {
    feedError = true;
    updateFeed();
    updateLimitControls();
  }
  if (!finished && !matchFailed) setTimeout(() => poll(offset), 125);
}

if (match.session) {
  const badge = byId("session-badge");
  badge.hidden = false;
  if (ownTeam) {
    badge.classList.add(ownTeam);
    badge.textContent = `Game · Your team is ${teams[ownTeam].label}`;
  } else {
    badge.textContent = "Practice";
  }
}
if (match.setup_url) {
  const link = byId("setup-link");
  link.href = match.setup_url;
  link.hidden = false;
}
if (livePage) poll();
else {
  initialLimits = initialLimitValues(match.limits);
  initializeTeams();
  frames.forEach(addRoleSample);
}
new ResizeObserver(resizeCanvas).observe(canvas);
new ResizeObserver(() => updateEventTrack(frames[frameIndex]?.t ?? 0)).observe(eventScroll);
render();
