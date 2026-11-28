"use strict";

const match = JSON.parse(document.getElementById("match-data").textContent);
const { frames = [], keys = [], positions = [], events = [] } = match;
const livePage = match.live === true;
const ownTeam = match.session?.team;
const kickFlightLimit = 2;
const kickResultWindow = 1.25;
const byId = (id) => document.getElementById(id);
const canvas = byId("pitch");
const pitchLabel = canvas.getAttribute("aria-label");
const ctx = canvas.getContext("2d");
const scrub = byId("scrub");
const playButton = byId("play");
const playbackRateSelect = byId("playback-rate");
const previousButton = byId("prev");
const nextButton = byId("next");
const liveButton = byId("go-live");
const motionSettings = byId("motion-settings");
const clipToggle = byId("clip-toggle");
const clipPanel = byId("clip-panel");
const clipDownload = byId("clip-download");
const clipStatus = byId("clip-status");
const clipNote = byId("clip-note");
const changeCreate = byId("change-create");
const forecastToggle = byId("forecast-toggle");
const forecastSummary = byId("forecast-summary");
const forecastError = byId("forecast-error");
const forecastReplay = byId("forecast-replay");
const forecastViewpoint = byId("forecast-viewpoint");
const teamPaths = match.forecast?.team_paths;
const replayQuery = new URLSearchParams(location.search);
const requestedViewpoint = replayQuery.get("view");
const forecastLabel = teamPaths ? "Context + plans" : "Kalman";
byId("forecast-model-key").textContent = forecastLabel;
forecastViewpoint.hidden = !teamPaths;
forecastViewpoint.value = ["blue", "yellow"].includes(requestedViewpoint)
  ? requestedViewpoint : ownTeam ?? "blue";
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
  Interceptor: "Meet the opposing pass and secure the ball.",
  Carrier: "Control the ball and advance the attack.",
  Receiver: "Move into the passing lane and bring the pass under control.",
  "Spin outlet": "Hold the outlet for the spin pass.",
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
  "Kickoff receiver": "Wait for the kickoff pass, then receive it.",
  "Kickoff support": "Hold a support position until the ball is in play.",
  "Stand clear": "Keep clear of the ball during the restart.",
  Pass: "Take the awarded free kick as a pass.",
  Receive: "Move into position for the free-kick pass.",
  hold: "Hold position for the referee.",
};
const roleActions = {
  Keeper: "Protect goal", Chaser: "Recover the ball", Presser: "Close down carrier",
  Interceptor: "Meet the pass", Carrier: "Carry the ball", Receiver: "Cushion the pass",
  "Spin outlet": "Hold passing lane", "Left wing": "Offer an outlet", "Right wing": "Offer an outlet",
  Anchor: "Stay behind the ball", Marker: "Track an opponent", Screen: "Block the middle",
  Cover: "Protect space behind", "Guard box": "Cover the return lane", "Block outlet": "Cover the outlet",
  Contain: "Drop goal-side", Outlet: "Offer an outlet", kickoff: "Set up for kickoff",
  Kickoff: "Take kickoff", "Kickoff receiver": "Receive kickoff", "Kickoff support": "Hold formation",
  "Stand clear": "Keep clear", Pass: "Take the free kick", Receive: "Receive the free kick", hold: "Hold position",
};
const roleColors = {
  Keeper: "#b4b6f1", Chaser: "#84dec5", Presser: "#ffae94",
  Interceptor: "#73d4cb",
  Carrier: "#ace38c", Receiver: "#86d4fa", "Spin outlet": "#86d4fa", "Left wing": "#8cb0ff", "Right wing": "#b7a3f4",
  Anchor: "#c1d69f", Marker: "#efbf87", Screen: "#ebd18f", Cover: "#edb1a0",
  "Guard box": "#d0a9cf", "Block outlet": "#d9b29b", Contain: "#f8a19e", Outlet: "#97dfb2",
  kickoff: "#b8c7d5", Kickoff: "#9ec5e9", "Kickoff receiver": "#86d4fa", "Kickoff support": "#b7a3f4", "Stand clear": "#9daebc",
  Pass: "#9fcce6", Receive: "#95dcea", hold: "#a0afc4",
};
let frameIndex = match.intro?.frame ?? 0;
let sourceHeader = match;
let clipStart = null;
let clipEnd = null;
let clipReview = null;
let requestingChange = false;
let branchPending = false;
let selected = match.intro?.robot ?? 0;
let forecastEnabled = Boolean(match.forecast && replayQuery.get("forecast") === "1");
if (!livePage && replayQuery.has("frame")) {
  const frame = Number(replayQuery.get("frame"));
  if (Number.isInteger(frame) && frame >= 0 && frame < frames.length) frameIndex = frame;
} else if (forecastEnabled) {
  const valid = match.forecast.states.findIndex((row, index) => row && index >= frameIndex);
  if (valid >= 0) frameIndex = valid;
}
if (!livePage && replayQuery.has("robot")) {
  const robot = keys.findIndex((_, robot) => robotName(robot) === replayQuery.get("robot"));
  if (robot >= 0) selected = robot;
}
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
let playing = false;
let playbackRate = Number(playbackRateSelect.value);
let playbackStart = 0;
let animationStart = 0;
let animationFrame = 0;
let canvasWidth = 0;
let canvasHeight = 0;
const teamEnabled = { blue: true, yellow: true };
const robotEnabled = [];
const overlayEnabled = new Set(match.intro?.overlays ?? []);
const lineup = [];
const teamPlans = {};
const filterButtons = [];
const teamButtons = {};
const motionControls = createLimitControls(byId("motion-limits"), match.limit_ranges, changeLimit);
const review = createMatchReview({
  frames, keys, positions, events, robotName, roleName, robotTask, timeText: replayTime, seek, render,
  selectedRobot: () => selected, openClip: openMomentClip,
});

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

function replayTime(seconds, precision = 1) {
  const scale = 10 ** precision;
  const ticks = Math.round(seconds * scale);
  const minutes = Math.floor(ticks / (60 * scale));
  const wholeSeconds = String(Math.floor(ticks / scale) % 60).padStart(2, "0");
  const fraction = String(ticks % scale).padStart(precision, "0");
  return `${minutes}:${wholeSeconds}.${fraction}`;
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

function addLimitSample(frame) {
  limitsByFrame.push(frame.limits || limitsByFrame.at(-1) || initialLimits);
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

function updateClipRange() {
  for (const [id, tick] of [["clip-start", clipStart], ["clip-end", clipEnd]]) {
    byId(id).textContent = tick === null ? "—" : replayTime(frames[tick].t);
  }
  const valid = clipStart !== null && clipEnd !== null && clipEnd > clipStart;
  const containsDecision = valid && clipReview && clipStart <= clipReview.tick && clipReview.tick <= clipEnd;
  const hasRequest = Boolean(containsDecision && byId("change-title").value.trim() && clipNote.value.trim());
  clipDownload.disabled = requestingChange ? !hasRequest : !valid;
  changeCreate.disabled = branchPending || !requestingChange || !hasRequest;
  if (requestingChange && !containsDecision) {
    clipStatus.textContent = "Keep the decision frame inside the evidence range.";
  } else if (requestingChange && !hasRequest) {
    clipStatus.textContent = "Enter a title and the decision you want the team to make.";
  } else if (valid) {
    const duration = frames[clipEnd].t - frames[clipStart].t;
    clipStatus.textContent = `${duration.toFixed(2)} s · ${clipEnd - clipStart + 1} samples`;
  } else {
    clipStatus.textContent = clipStart === null ? "Scrub or play to the start, then mark it."
      : clipEnd === null ? "Move to the end of the moment, then mark it."
      : "Mark an end after the start.";
  }
}

function openMomentClip(anchor, note, requestChange = false) {
  seek(anchor.tick);
  clipReview = anchor;
  clipStart = Math.max(0, frames.findLastIndex((frame) => frame.t <= anchor.time_s - 2));
  const end = frames.findIndex((frame) => frame.t >= anchor.time_s + 2);
  clipEnd = end < 0 ? frames.length - 1 : end;
  requestingChange = requestChange;
  byId("change-fields").hidden = !requestChange;
  changeCreate.hidden = !requestChange || !match.setup_url || location.protocol === "file:";
  clipDownload.textContent = requestChange ? "Download request" : "Download JSON";
  clipDownload.classList.toggle("primary", changeCreate.hidden);
  byId("clip-note-label").textContent = requestChange ? "Desired decision or outcome" : "Observation or suggested play";
  if (requestChange) {
    const coachingPlay = anchor.coaching_play;
    byId("change-title").value = coachingPlay?.objective?.slice(0, 100) || `Decision at ${replayTime(anchor.time_s)}`;
    byId("change-team").value = coachingPlay?.team || anchor.team || keys[anchor.robot]?.[0] || "blue";
    byId("change-acceptance").value = "";
  }
  clipNote.value = note;
  clipPanel.hidden = false;
  clipToggle.setAttribute("aria-expanded", "true");
  updateClipRange();
  clipNote.focus();
}

function telemetryClip() {
  const matchId = location.pathname.match(/\/matches\/([a-f0-9]{12})\/$/)?.[1] ?? null;
  const eventData = ([tick, label, kind, team, details]) => (
    { tick, time_s: frames[tick]?.t ?? null, label, kind, team, details: details ?? null }
  );
  const previousKick = events.findLast(([tick, , , , details]) => tick < clipStart && details?.origin);
  const clip = {
    format: "roboturk.telemetry_clip.v1",
    source: {
      match_id: matchId,
      seed: sourceHeader.seed ?? null,
      sample_period_s: sourceHeader.dt ?? null,
      team_strategies: match.team_strategies ?? match.session?.strategies ?? null,
      ...(match.controller_versions && { controller_versions: match.controller_versions }),
      ...(match.replay_moment && { replay_moment: match.replay_moment }),
    },
    range: {
      start_tick: clipStart,
      end_tick: clipEnd,
      start_s: frames[clipStart].t,
      end_s: frames[clipEnd].t,
    },
    note: clipNote.value.trim(),
    robot_keys: keys,
    position_fields: { ball: ["x_m", "y_m"], robot: ["x_m", "y_m", "heading_rad"] },
    command_fields: ["forward_m_s", "left_m_s", "turn_rad_s", "dribbler", "kick_m_s"],
    samples: frames.slice(clipStart, clipEnd + 1).map((frame, offset) => {
      const tick = clipStart + offset;
      return { tick, position: positions[tick], state: { ...frame, limits: limitsByFrame[tick] ?? null } };
    }),
    preceding_kick: previousKick ? eventData(previousKick) : null,
    events: events.filter(([tick]) => clipStart <= tick && tick <= clipEnd).map(eventData),
  };
  if (clipReview && clipStart <= clipReview.tick && clipReview.tick <= clipEnd) {
    clip.review = clipReview;
  }
  return clip;
}

function decisionRequest(clip) {
  return {
    format: "roboturk.decision_change_request.v1",
    title: byId("change-title").value.trim(),
    team: byId("change-team").value,
    intent: clip.note,
    acceptance: byId("change-acceptance").value.trim(),
    evidence: clip,
  };
}

function downloadClip() {
  if (clipDownload.disabled) return;
  const clip = telemetryClip();
  const value = requestingChange ? decisionRequest(clip) : clip;
  const url = URL.createObjectURL(new Blob([JSON.stringify(value, null, 2)], { type: "application/json" }));
  const link = document.createElement("a");
  link.href = url;
  const source = clip.source.match_id ?? clip.source.seed ?? "match";
  link.download = requestingChange ? `hoca-change-${source}-${clip.review.tick}.json`
    : `hoca-${source}-${clipStart}-${clipEnd}.json`;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  clipStatus.textContent = requestingChange ? "Downloaded the change request with its replay evidence."
    : `Exported ${clip.samples.length} samples as JSON.`;
}

async function createDecisionBranch() {
  if (changeCreate.disabled) return;
  const request = decisionRequest(telemetryClip());
  branchPending = true;
  changeCreate.disabled = true;
  clipStatus.textContent = "Preparing the candidate workspace…";
  let message;
  try {
    const response = await fetch("/decision-changes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request),
    });
    const payload = await response.json();
    message = response.ok ? `${payload.created ? "Created" : "Found"} ${payload.branch} in ${payload.worktree}`
      : payload.error;
  } catch {
    message = "Could not reach the local candidate workspace service. You can still download the request.";
  } finally {
    branchPending = false;
    updateClipRange();
    clipStatus.textContent = message;
  }
}

function choose(robot) {
  selected = robot;
  render();
}

function makeRosterColumn(team, robot) {
  const column = document.createElement("article");
  column.className = "roster-player";
  const element = document.createElement("button");
  element.type = "button";
  element.className = "roster-id";
  element.textContent = robotName(robot);
  const role = document.createElement("strong");
  role.className = "roster-role";
  const task = document.createElement("p");
  task.className = "roster-task";
  const detail = document.createElement("p");
  detail.className = "roster-detail";
  column.append(element, role, task, detail);
  element.addEventListener("click", () => {
    if (overlayEnabled.has(robot)) overlayEnabled.delete(robot);
    else overlayEnabled.add(robot);
    choose(robot);
  });
  teamPlans[team].roster.append(column);
  return { column, element, role, task, detail };
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
    const plan = byId(`${team}-plan`);
    plan.append(byId("team-plan-template").content.cloneNode(true));
    plan.querySelector(".lineup-team").textContent = label;
    const lineupToggle = plan.querySelector(".lineup-toggle");
    const scroll = plan.querySelector(".roster-scroll");
    const roster = plan.querySelector(".roster");
    const overlay = plan.querySelector(".team-overlay");
    scroll.id = `${team}-lineup`;
    scroll.setAttribute("aria-label", `${label} robot IDs, roles, behaviors, and explanations`);
    lineupToggle.setAttribute("aria-controls", scroll.id);
    overlay.setAttribute("aria-label", `Toggle ${label} overlays`);
    overlay.title = `Toggle ${label} overlays`;
    teamPlans[team] = {
      mode: plan.querySelector('[data-plan="mode"]'),
      play: plan.querySelector('[data-plan="play"]'),
      step: plan.querySelector('[data-plan="step"]'),
      reason: plan.querySelector(".plan-reason"),
      side: plan.querySelector(".goal-side"),
      lineupToggle, roster, overlay,
    };
    lineupToggle.addEventListener("click", () => {
      scroll.hidden = !scroll.hidden;
      lineupToggle.setAttribute("aria-expanded", String(!scroll.hidden));
      lineupToggle.textContent = scroll.hidden ? "Show lineup" : "Hide lineup";
    });
    overlay.addEventListener("click", () => {
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
  keys.forEach(([team], robot) => {
    robotEnabled.push(true);
    lineup.push(makeRosterColumn(team, robot));
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
  for (const { lineupToggle, roster } of Object.values(teamPlans)) {
    lineupToggle.disabled = !roster.childElementCount;
    roster.style.setProperty("--players", roster.childElementCount);
  }
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

function robotTask(frame, robot) {
  const audit = frame.allocation?.[robot];
  const role = frame.roles[robot];
  if (audit && !audit.command && frame.mode && frame.mode !== "running") return "Hold for the referee";
  if (audit?.spin_turn && audit.command?.[4] > 0) {
    const action = audit.pass_to != null ? `pass to ${robotName(audit.pass_to)}` : "shot at goal";
    return `${audit.spin_turn.reverse ? "Reverse spin" : "Spin"} ${action}.`;
  }
  if (audit?.pass_to != null) {
    return `${audit.kick_elevation > 0 ? "Chip" : "Pass"} to ${robotName(audit.pass_to)}.`;
  }
  if (audit?.kick_elevation > 0) return "Chip to the receiver.";
  if (audit?.command?.[4] > 0 && audit.task === "Line up a shot") return "Shoot at goal.";
  return audit?.task || roleActions[role] || "—";
}

function updateLineup(frame) {
  const highlighted = review.highlightedRobots();
  for (const [index, team] of Object.keys(teams).entries()) {
    const { mode, play, step, reason, side, overlay } = teamPlans[team];
    const roster = keys.flatMap(([color], robot) => color === team ? [robot] : []);
    const enabled = roster.filter((robot) => overlayEnabled.has(robot)).length;
    overlay.disabled = !roster.length;
    overlay.setAttribute("aria-pressed", enabled === 0 ? "false" : enabled === roster.length ? "true" : "mixed");
    const entry = frame.plays?.[index];
    const phase = frame.phases?.[index];
    const values = { mode: teamMode(phase), play: entry?.play, step: entry?.step };
    for (const label of [mode, play, step]) {
      const value = values[label.dataset.plan] || "—";
      if (label.textContent !== value) label.textContent = value;
    }
    const description = entry?.reason || "";
    if (reason.textContent !== description) reason.textContent = description;
    reason.hidden = !description;
    const goal = ((frame.sign ?? 1) > 0) === (team === "blue") ? "left" : "right";
    side.textContent = `Defends ${goal} goal`;
    play.title = [entry?.strategy, entry?.passing === "Preset default" ? null : entry?.passing].filter(Boolean).join(" · ");
  }
  lineup.forEach(({ column, element, role, task, detail }, robot) => {
    const raw = frame.roles[robot];
    const current = roleName(raw) || "—";
    const behavior = robotTask(frame, robot);
    const audit = frame.allocation?.[robot];
    const explanation = explain(frame, raw, audit, keys[robot][0]);
    const description = audit?.decision || roleTasks[raw] || explanation;
    if (role.textContent !== current) role.textContent = current;
    if (task.textContent !== behavior) task.textContent = behavior;
    if (detail.textContent !== description) detail.textContent = description;
    detail.title = description === explanation ? explanation : `${description}
${explanation}`;
    column.style.setProperty("--role-color", roleColors[raw] ?? "var(--text)");
    column.toggleAttribute("data-linked", highlighted.includes(robot));
    element.setAttribute("aria-label", `${robotName(robot)}, ${current}. ${behavior}. Toggle overlay`);
    element.setAttribute("aria-pressed", String(overlayEnabled.has(robot)));
    if (selected === robot) element.setAttribute("aria-current", "true");
    else element.removeAttribute("aria-current");
  });
}

function explain(frame, role, audit, team) {
  const phase = frame.phases?.[team === "blue" ? 0 : 1];
  if (audit?.basis === "keeper" || role === "Keeper") return "Robot 0 is the designated goalkeeper.";
  if (audit?.basis === "receiver" || role === "Receiver") return "The pass targets this robot, so the controller reserves it.";
  if (role === "Interceptor") return "The defending team has reassigned this robot to the pass.";
  if (role === "Spin outlet") return "The carrier has selected this robot for the spin pass.";
  if (audit?.basis === "carrier" || role === "Carrier") return "This robot has confirmed possession.";
  if (audit?.basis === "contain" || role === "Contain") return "Both robots gripped the ball, so this player drops goal-side.";
  if (audit?.basis === "roster_cost" || audit?.basis === "role_priority") {
    const switchNote = audit.switch_cost > 0 && audit.allocator_previous
      ? ` Switching from ${roleName(audit.allocator_previous)} added ${audit.switch_cost.toFixed(2)} to the cost.` : "";
    const assignment = audit.basis === "role_priority"
      ? "Roles fill in order by travel and switch cost."
      : "The lineup minimizes travel and role changes.";
    return `${assignment} ${phaseReasons[phase] || ""}${switchNote}`;
  }
  if (role === "kickoff" || role === "Kickoff") return "The referee awarded a kickoff. This robot has a restart assignment.";
  if (role === "Kickoff receiver") return "The team controller chose this robot for the kickoff pass.";
  if (role === "Kickoff support") return "The team keeps its support players in formation until the ball is in play.";
  if (role === "hold" || role === "Stand clear" || frame.mode && frame.mode !== "running") return "The referee limits movement during this stoppage or restart.";
  if (role === "Pass" || role === "Receive") return "The free-kick controller chose the passer and a receiving outlet.";
  return "Recorded lineup assignment.";
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
  byId("motion-limit-section").hidden = !limits;
  for (const [name, value] of pendingLimits) {
    if (limits?.blue[name] === value) pendingLimits.delete(name);
  }
  motionControls.show(limits, editable, editable ? pendingLimits : undefined);
}

function render() {
  updateFeed();
  updateFilters();
  updateLimitControls();
  const frame = frames[frameIndex];
  playButton.disabled = !frame || (livePage && following && !finished);
  playbackRateSelect.disabled = playButton.disabled;
  scrub.disabled = !frame;
  clipToggle.disabled = !frame;
  forecastToggle.hidden = !match.forecast;
  forecastToggle.setAttribute("aria-pressed", String(forecastEnabled));
  forecastSummary.hidden = !forecastEnabled;
  previousButton.disabled = nextButton.disabled = !frame;
  if (!frame) {
    review.update(frameIndex, false, sourceHeader);
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
  byId("replay-time").textContent = `${replayTime(frame.t)} / ${replayTime(frames.at(-1).t)}`;
  scrub.max = frames.length - 1;
  scrub.value = frameIndex;
  scrub.setAttribute("aria-valuetext", `${replayTime(frame.t)} elapsed`);
  review.update(frameIndex, !livePage || (finished && !matchFailed), sourceHeader);
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

function latestKick() {
  let action;
  let result;
  for (const event of events) {
    const [tick, , kind, , details] = event;
    if (!details || tick > frameIndex) continue;
    if (kind === "kick_trace") {
      if (action && action[0] === details.kick_tick && action[4].actor === details.actor) result = details;
    } else if (details.origin) {
      action = event;
      result = undefined;
    }
  }
  if (!action) return null;
  const [tick, , kind, team, details] = action;
  return { launch: { ...details, tick, kind, team }, result };
}

function updateKickReadout(kick) {
  let text = "";
  if (kick) {
    const { launch, result } = kick;
    const turn = launch.spin_turn;
    const action = launch.elevation > 0 ? "chip" : launch.kind;
    const spin = turn ? `${turn.reverse ? "reverse " : ""}spin ` : "";
    const receiver = launch.receiver == null ? "" : ` to ${robotName(launch.receiver)}`;
    const label = passLabel(launch.kind, launch, robotName) ?? `${robotName(launch.actor)} ${spin}${action}${receiver}`;
    text = `${label} · ${launch.speed.toFixed(1)} m/s launch`;
    if (turn) text += ` · ${turn.degrees.toFixed(0)}° ${turn.direction} rotation`;
    if (launch.elevation > 0) text += ` · ${Math.round(launch.elevation * 180 / Math.PI)}° loft`;
    if (result?.outcome === "goal mouth" || result?.outcome === "wide") {
      text += ` · ${result.speed.toFixed(2)} m/s at goal line · ${Math.abs(result.point[1]).toFixed(2)} m from center · ${result.point[2].toFixed(2)} m high`;
      if (result.outcome === "wide") text += " · wide";
    } else if (result && launch.elevation > 0) {
      text += ` · ${result.peak_z.toFixed(2)} m peak`;
    }
    if (result?.outcome === "deflected") text += " · deflection detected";
  }
  const readout = byId("kick-readout");
  if (readout.textContent !== text) {
    readout.textContent = text;
    readout.title = text;
  }
}

function drawKickTrail(launch, result, color) {
  const track = [launch.origin];
  for (let i = launch.tick + 1; i <= frameIndex; i += 1) {
    track.push([positions[i][0], positions[i][1], frames[i].ball_z ?? 0]);
  }
  if (result) track.push(result.point);
  const path = track.map(([x, y]) => project(x, y));
  if (launch.elevation > 0) {
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
}

function drawOverlays(frame, points, kick) {
  for (const robot of overlayEnabled) {
    const audit = frame.allocation?.[robot];
    if (!isVisible(robot) || !audit?.target || !audit.command
      || ["hold", "Stand clear"].includes(frame.roles[robot])) continue;
    const offset = 2 + 3 * robot;
    const origin = project(points[offset], points[offset + 1]);
    const target = project(...audit.target);
    if (Math.hypot(target[0] - origin[0], target[1] - origin[1]) <= 12) continue;
    const color = teams[keys[robot][0]].pale;
    strokePath([origin, target], `${color}a0`, 1.3, [5, 5]);
    traceCircle(...target, 4, null, color, 1.4);
  }
  if (!kick || !overlayEnabled.has(kick.launch.actor) || !isVisible(kick.launch.actor)) return;
  const { launch, result } = kick;
  const color = teams[launch.team].pale;
  const showTrail = result ? frameIndex === result.tick : frame.t - frames[launch.tick].t <= kickFlightLimit;
  if (showTrail) drawKickTrail(launch, result, color);
  if (result?.outcome === "goal mouth" && frame.t - frames[result.tick].t <= kickResultWindow) {
    const [x, y] = project(...result.point);
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
}

function drawForecast() {
  if (!forecastEnabled) return;
  const row = match.forecast.states[frameIndex];
  const name = robotName(selected);
  if (!isVisible(selected) || !row) {
    forecastError.textContent = !isVisible(selected) ? `Show ${name} to see its path.`
      : "Choose a frame with a full second of open play ahead.";
    return;
  }
  const offset = 2 + selected * 3;
  const [vx, vy, kx, ky, kvx, kvy] = row.slice(selected * 6, (selected + 1) * 6);
  const horizon = match.forecast.horizon;
  const sample = horizon / match.dt;
  const whole = Math.floor(sample + 1e-9);
  const actual = positions.slice(frameIndex, frameIndex + whole + 1)
    .map((points) => points.slice(offset, offset + 2));
  if (sample - whole > 1e-9) {
    const next = positions[frameIndex + whole + 1];
    actual.push(actual.at(-1).map((value, axis) =>
      value + (sample - whole) * (next[offset + axis] - value)));
  }
  const [x, y] = actual[0];
  const current = [[x, y], [x + horizon * vx, y + horizon * vy]];
  let modelPath = [[kx, ky], [kx + horizon * kvx, ky + horizon * kvy]];
  const teamRow = teamPaths?.[forecastViewpoint.value]?.[frameIndex];
  if (teamRow) {
    modelPath = [[x, y]];
    const width = teamRow.length / keys.length;
    const start = selected * width;
    for (let i = start; i < start + width; i += 2) modelPath.push([teamRow[i], teamRow[i + 1]]);
  }
  const style = getComputedStyle(forecastSummary);
  for (const [kind, path, dash] of [
    ["actual", actual, []], ["current", current, [8, 4]], ["model", modelPath, [2, 4]],
  ]) {
    const color = style.getPropertyValue(`--forecast-${kind}`).trim();
    const points = path.map(([px, py]) => project(px, py));
    strokePath(points, color, 2.1, dash);
    traceCircle(...points.at(-1), 4, "#16382e", color, 1.6);
  }
  const error = (path) => Math.hypot(...path.at(-1).map((v, axis) => v - actual.at(-1)[axis]));
  const viewpoint = forecastViewpoint.value;
  const relationship = teamRow ? (keys[selected][0] === viewpoint ? " teammate" : " opponent") : "";
  forecastError.textContent = `${name}${relationship} at +${horizon} s · Current ${error(current).toFixed(2)} m · ${forecastLabel} ${error(modelPath).toFixed(2)} m`;
}

function drawField() {
  const frame = frames[frameIndex];
  const kick = frame ? latestKick() : null;
  updateKickReadout(kick);
  if (!canvasWidth || !canvasHeight) return;
  const k = scale();
  const points = positions[frameIndex];
  const highlighted = review.highlightedRobots();
  const label = highlighted.length ? `${pitchLabel} Linked event highlights ${highlighted.map(robotName).join(" and ")}.` : pitchLabel;
  if (canvas.getAttribute("aria-label") !== label) canvas.setAttribute("aria-label", label);
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
  drawOverlays(frame, points, kick);
  drawForecast();
  keys.forEach(([team], robot) => {
    const linked = highlighted.includes(robot);
    if (!linked && !isVisible(robot)) return;
    const offset = 2 + 3 * robot;
    const [x, y] = project(points[offset], points[offset + 1]);
    const yaw = points[offset + 2];
    const style = teams[team];
    const radius = Math.max(7, Math.min(13, .15 * k));
    if (linked) {
      traceCircle(x, y, radius + 8, null, `${style.pale}69`, 1.5);
      traceCircle(x, y, radius + 4.5, null, style.pale, 2.5);
    } else if (!highlighted.length && robot === selected) {
      traceCircle(x, y, radius + 4, null, style.pale, 2);
    }
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

function playbackTime(now) {
  return playbackStart + (now - animationStart) * playbackRate / 1000;
}

function animate(now) {
  if (!playing) return;
  const replayPosition = playbackTime(now);
  let next = frameIndex;
  while (next + 1 < frames.length && frames[next + 1].t <= replayPosition) next += 1;
  if (next !== frameIndex) goTo(next);
  if (next >= frames.length - 1) pause();
  else animationFrame = window.requestAnimationFrame(animate);
}

playButton.addEventListener("click", () => {
  if (!frames.length) return;
  if (playing) { pause(); return; }
  if (frameIndex >= frames.length - 1) goTo(0);
  review.collapse();
  playing = true;
  playbackStart = frames[frameIndex].t;
  animationStart = performance.now();
  playButton.textContent = "Pause";
  render();
  animationFrame = window.requestAnimationFrame(animate);
});

playbackRateSelect.addEventListener("change", () => {
  if (playing) {
    const now = performance.now();
    playbackStart = playbackTime(now);
    animationStart = now;
  }
  playbackRate = Number(playbackRateSelect.value);
});

for (const disclosure of [byId("field-info"), motionSettings]) {
  document.addEventListener("click", (event) => {
    if (disclosure.open && !disclosure.contains(event.target)) disclosure.open = false;
  });
  disclosure.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && disclosure.open) {
      disclosure.open = false;
      disclosure.querySelector("summary").focus();
    }
  });
}

scrub.addEventListener("input", () => seek(Number(scrub.value)));
forecastToggle.addEventListener("click", () => {
  forecastEnabled = !forecastEnabled;
  render();
});
forecastViewpoint.addEventListener("change", drawField);
clipToggle.addEventListener("click", () => {
  clipPanel.hidden = !clipPanel.hidden;
  clipToggle.setAttribute("aria-expanded", String(!clipPanel.hidden));
  if (!clipPanel.hidden) updateClipRange();
});
byId("clip-mark-start").addEventListener("click", () => {
  clipStart = frameIndex;
  updateClipRange();
});
byId("clip-mark-end").addEventListener("click", () => {
  clipEnd = frameIndex;
  updateClipRange();
});
clipDownload.addEventListener("click", downloadClip);
changeCreate.addEventListener("click", createDecisionBranch);
for (const field of [byId("change-title"), clipNote]) field.addEventListener("input", updateClipRange);
previousButton.addEventListener("click", () => review.jump(-1));
nextButton.addEventListener("click", () => review.jump(1));
liveButton.addEventListener("click", () => {
  pause();
  review.collapse();
  following = true;
  goTo(frames.length - 1);
});
byId("solo").addEventListener("click", () => { solo = !solo; render(); });
byId("show-all").addEventListener("click", () => {
  solo = false;
  teamEnabled.blue = teamEnabled.yellow = true;
  robotEnabled.fill(true);
  render();
});
function updateFeed() {
  if (!livePage) return;
  let [state, label] = ["waiting", "Waiting for Webots"];
  if (matchFailed) [state, label] = ["offline", "Match failed to run"];
  else if (finished) [state, label] = ["replay", "Match finished"];
  else if (feedError) [state, label] = ["offline", "Reconnecting"];
  else if (frames.length) [state, label] = following ? ["live", "Live · Webots"] : ["review", "Reviewing live match"];
  byId("feed-state").dataset.state = state;
  byId("feed-label").textContent = label;
  forecastReplay.hidden = !finished || matchFailed;
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
        sourceHeader = item.header;
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
        addLimitSample(item.frame);
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
if (match.controller_versions) {
  const { build, teams: releases } = match.controller_versions;
  const rows = Object.entries(teams).map(([color, { label }]) => {
    const release = releases[color];
    return [label, `${release.name} · v${release.version}`];
  });
  if (build) {
    const checkout = build.working_copy_sha256 ? ` · local ${build.working_copy_sha256.slice(0, 8)}` : "";
    rows.push(["Build", `${build.number} · ${build.commit.slice(0, 8)}${checkout}`]);
  }
  for (const [label, value] of rows) {
    const row = document.createElement("div");
    const term = document.createElement("dt");
    term.textContent = label;
    const detail = document.createElement("dd");
    detail.textContent = value;
    row.append(term, detail);
    byId("recorded-version-list").append(row);
  }
  byId("recorded-versions").hidden = false;
}
if (livePage) poll();
else {
  initialLimits = initialLimitValues(match.limits);
  initializeTeams();
  frames.forEach(addLimitSample);
}
new ResizeObserver(resizeCanvas).observe(canvas);
render();
