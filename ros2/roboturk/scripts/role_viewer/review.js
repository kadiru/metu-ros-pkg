"use strict";

function indexReviewMarkers(frames, keys, events, robotName, roleName) {
  const markers = [];
  const sideNames = { blue: "Blue", yellow: "Yellow" };
  const add = (tick, kind, label, teams, details = {}) => markers.push({ tick, kind, label, teams, ...details });
  let lastOwner = null;
  let ballState = null;
  for (const [tick, frame] of frames.entries()) {
    const previous = frames[tick - 1];
    const inPlay = !frame.mode || frame.mode === "running";
    if (!inPlay || previous?.restart !== frame.restart) {
      lastOwner = null;
      ballState = null;
    }
    if (inPlay) {
      const owner = keys[frame.owner] ? frame.owner : null;
      const state = frame.roles.includes("Receiver") ? "pass" : owner === null ? "loose" : "owned";
      if (state === "owned" && (owner !== lastOwner || ballState !== "owned")) {
        const team = keys[owner][0];
        const before = lastOwner === null ? null : keys[lastOwner][0];
        const action = before && before !== team ? "Turnover"
          : lastOwner === null || owner === lastOwner ? "Recovery" : "Possession";
        const label = lastOwner !== null && lastOwner !== owner
          ? `${action} · ${robotName(lastOwner)} → ${robotName(owner)}`
          : `${action} · ${robotName(owner)} secures the ball`;
        add(tick, "possession", label, before && before !== team ? [before, team] : [team],
          { action, from: lastOwner, to: owner });
        lastOwner = owner;
      } else if (state === "loose" && ballState && ballState !== "loose") {
        add(tick, "possession", "Loose ball", lastOwner === null ? [] : [keys[lastOwner][0]],
          { action: "Loose ball", from: lastOwner, to: null });
      }
      ballState = state;
    }
    if (!previous) continue;
    for (const [index, team] of ["blue", "yellow"].entries()) {
      const next = frame.plays?.[index];
      const prior = previous.plays?.[index];
      if (prior && next && (next.strategy !== prior.strategy || next.play !== prior.play || next.step !== prior.step)) {
        add(tick, "decision", [sideNames[team], next.play, next.step].filter(Boolean).join(" · "), [team],
          { strategy: next.strategy, play: next.play, step: next.step, reason: next.reason });
      }
    }
    for (const [robot, [team]] of keys.entries()) {
      const role = frame.roles[robot], before = previous.roles[robot];
      if (role && role !== before) {
        add(tick, "role", `${robotName(robot)} · ${roleName(before) || "Unassigned"} → ${roleName(role)}`,
          [team], { robot, from: before, to: role });
      }
    }
  }
  for (const [tick, label, type, team] of events) {
    if (frames[tick] && label && type !== "kick_trace") {
      add(tick, "event", label, team ? [team] : [], { type });
    }
  }
  return markers;
}

function createMatchReview({
  frames, keys, positions, events, robotName, roleName, timeText, seek, render, selectedRobot, openClip,
}) {
  const el = (id) => document.getElementById(id);
  const toggle = el("review-toggle");
  const panel = el("review-panel");
  const track = el("review-track");
  const moment = el("review-moment");
  const filters = el("review-filters");
  const filterToggle = el("review-filter-toggle");
  const team = el("review-team");
  const search = el("review-search");
  const choices = [...filters.querySelectorAll('input[type="checkbox"]')];
  const details = el("review-details");
  const noteButton = el("review-note-toggle");
  const noteForm = el("review-note-form");
  const noteField = el("review-note");
  const playButton = el("review-play-toggle");
  const playFields = el("review-coaching-play");
  const playTeam = el("review-play-team");
  const playObjective = el("review-play-objective");
  const playPlayers = el("review-play-players");
  const noteStatus = el("review-status");
  const previous = el("prev");
  const next = el("next");
  const query = new URLSearchParams(location.search);
  let active = query.get("review") === "1";
  if (["blue", "yellow"].includes(query.get("team"))) team.value = query.get("team");
  let tick = 0;
  let editing = null;
  let lastRenderedTick = null;
  let storageKey;
  let markers;
  let groups = new Map();
  let ticks = [];
  let notes = new Map();
  let players = [];

  function setStatus(message) {
    noteStatus.hidden = !message;
    noteStatus.textContent = message;
  }

  function loadNotes(source) {
    const identity = location.pathname.match(/\/matches\/([a-f0-9]{12})\//)?.[1] ?? [
      source.seed, frames.length, frames.at(-1).t, frames.at(-1).score,
      positions[0]?.slice(0, 8), positions.at(-1)?.slice(0, 8),
    ];
    storageKey = `hoca.review.notes.v1:${JSON.stringify(identity)}`;
    try {
      const rows = JSON.parse(localStorage.getItem(storageKey) ?? "[]");
      for (const note of rows) {
        if (Number.isInteger(note.tick) && frames[note.tick] && typeof note.text === "string") {
          notes.set(note.tick, note);
        }
      }
    } catch {
      // The session and clip export still work when storage is unavailable.
    }
  }

  function refreshMarkers() {
    const kinds = new Set(choices.filter((input) => input.checked).map((input) => input.value));
    const term = search.value.trim().toLowerCase();
    const notesAsMarkers = [...notes.values()].map((note) => ({
      tick: note.tick, kind: "note", label: note.text || note.play?.objective,
      teams: note.team ? [note.team] : [],
      searchText: note.play ? [note.play.objective, ...note.play.players.map((player) => `${player.id} ${player.action}`)].join(" ") : "",
    }));
    groups = new Map();
    for (const marker of [...notesAsMarkers, ...markers]) {
      if (!kinds.has(marker.kind) || team.value && marker.teams.length && !marker.teams.includes(team.value)) continue;
      if (term && !`${timeText(frames[marker.tick].t)} ${marker.label} ${marker.searchText ?? ""}`.toLowerCase().includes(term)) continue;
      if (!groups.has(marker.tick)) groups.set(marker.tick, []);
      groups.get(marker.tick).push(marker);
    }
    ticks = Array.from(groups.keys()).sort((a, b) => a - b);
    const options = document.createDocumentFragment();
    options.append(new Option(ticks.length ? `${ticks.length} marked moments` : "No matching moments", ""));
    const marks = document.createDocumentFragment();
    for (const index of ticks) {
      const rows = groups.get(index);
      const label = rows[0].label.replace(/\s+/g, " ");
      const caption = label.length > 86 ? `${label.slice(0, 85)}…` : label;
      options.append(new Option(`${timeText(frames[index].t)} · ${caption}${rows.length > 1 ? ` (+${rows.length - 1})` : ""}`, index));
      const mark = document.createElement("span");
      mark.className = "review-mark";
      mark.dataset.tick = index;
      mark.dataset.kind = rows[0].kind;
      mark.style.setProperty("--mark-position", `${100 * index / Math.max(1, frames.length - 1)}%`);
      marks.append(mark);
    }
    moment.replaceChildren(options);
    track.replaceChildren(marks);
    moment.disabled = !ticks.length;
    lastRenderedTick = null;
    showMoment();
  }

  function showMoment() {
    if (lastRenderedTick === tick) return;
    lastRenderedTick = tick;
    moment.value = groups.has(tick) ? String(tick) : "";
    track.querySelector(".is-current")?.classList.remove("is-current");
    track.querySelector(`[data-tick="${tick}"]`)?.classList.add("is-current");
    const rows = [...(groups.get(tick) ?? [])];
    const note = notes.get(tick);
    if (note && !rows.some(({ kind }) => kind === "note")) {
      rows.unshift({ kind: "note", label: note.text || note.play?.objective });
    }
    details.replaceChildren(...rows.map(({ kind, label }) => {
      const row = document.createElement("li");
      row.dataset.kind = kind;
      row.textContent = label;
      return row;
    }));
    details.hidden = !rows.length;
    noteButton.textContent = note?.text ? "Edit note" : "Add note";
    playButton.textContent = note?.play ? `Edit play · ${note.play.players.length} robots` : "Coach a play";
  }

  function noteOpen(value) {
    noteForm.hidden = !value;
    noteButton.setAttribute("aria-expanded", String(value));
    if (!value) {
      editing = null;
      playFields.open = false;
      playButton.setAttribute("aria-expanded", "false");
    }
  }

  function fillPlayForm(play) {
    playTeam.value = play?.team ?? editing.team ?? keys[editing.robot][0];
    playObjective.value = play?.objective ?? "";
    players = keys.map(([color], robot) => {
      const id = robotName(robot);
      const role = roleName(frames[editing.tick].roles[robot]) || "Unassigned";
      const saved = play?.players.find((player) => player.robot === robot);
      const row = document.createElement("div");
      row.className = "review-play-player";
      row.dataset.team = color;
      row.hidden = color !== playTeam.value;
      const label = document.createElement("label");
      const name = document.createElement("strong");
      name.textContent = id;
      const recorded = document.createElement("span");
      recorded.className = "recorded-role";
      recorded.textContent = role;
      const include = document.createElement("input");
      include.type = "checkbox";
      include.checked = Boolean(saved);
      label.append(include, name, recorded);
      const action = document.createElement("input");
      action.type = "text";
      action.maxLength = 200;
      action.value = saved?.action ?? "";
      action.hidden = !include.checked;
      action.placeholder = "Proposed action";
      action.setAttribute("aria-label", `Proposed action for ${id}`);
      include.addEventListener("change", () => { action.hidden = !include.checked; });
      row.append(label, action);
      return { row, include, action, robot, id, recorded_role: role };
    });
    playPlayers.replaceChildren(...players.map(({ row }) => row));
  }

  toggle.addEventListener("click", () => {
    active = !active;
    noteOpen(false);
    render();
  });
  filterToggle.addEventListener("click", () => {
    filters.hidden = !filters.hidden;
    filterToggle.setAttribute("aria-expanded", String(!filters.hidden));
  });
  function filterChanged() {
    refreshMarkers();
    render();
  }
  for (const control of [team, ...choices]) control.addEventListener("change", filterChanged);
  search.addEventListener("input", filterChanged);
  moment.addEventListener("change", () => {
    if (moment.value !== "") seek(Number(moment.value));
  });
  track.addEventListener("click", (event) => {
    if (!ticks.length) return;
    const box = track.getBoundingClientRect();
    const point = (event.clientX - box.left) / box.width * (frames.length - 1);
    const closest = ticks.reduce((best, index) => Math.abs(index - point) < Math.abs(best - point) ? index : best);
    seek(closest);
  });
  function editNote(startWithPlay) {
    const trigger = startWithPlay ? playButton : noteButton;
    if (editing?.tick === tick && !noteForm.hidden) {
      editing.trigger = trigger;
      playFields.open = startWithPlay;
      (startWithPlay ? playObjective : noteField).focus();
      return;
    }
    seek(tick);
    const existing = notes.get(tick);
    editing = { tick, team: existing?.team ?? (team.value || null), robot: existing?.robot ?? selectedRobot(), trigger };
    noteField.value = existing?.text ?? "";
    el("review-note-label").textContent = `Coaching note · ${timeText(frames[tick].t)}`;
    fillPlayForm(existing?.play);
    noteOpen(true);
    playFields.open = startWithPlay;
    setStatus("");
    (startWithPlay ? playObjective : noteField).focus();
  }
  noteButton.addEventListener("click", () => editNote(false));
  playButton.addEventListener("click", () => editNote(true));
  playFields.addEventListener("toggle", () => {
    playButton.setAttribute("aria-expanded", String(playFields.open));
    noteForm.querySelector('[type="submit"]').textContent = playFields.open ? "Save play" : "Save note";
  });
  playTeam.addEventListener("change", () => {
    for (const { row } of players) row.hidden = row.dataset.team !== playTeam.value;
  });
  el("review-note-cancel").addEventListener("click", () => {
    const trigger = editing?.trigger ?? noteButton;
    noteOpen(false);
    trigger.focus();
  });
  noteForm.addEventListener("submit", (event) => {
    event.preventDefault();
    if (!editing) return;
    const text = noteField.value.trim();
    const objective = playObjective.value.trim();
    const lineup = players.filter(({ row, include }) => !row.hidden && include.checked)
      .map(({ robot, id, recorded_role, action }) => ({ robot, id, recorded_role, action: action.value.trim() }));
    const hasPlay = Boolean(objective || lineup.length);
    if (hasPlay && (!objective || lineup.length < 2 || lineup.length > 5 || lineup.some(({ action }) => !action))) {
      playFields.open = true;
      setStatus("Give the play an objective and each of the two to five selected teammates an action.");
      return;
    }
    if (!text && !hasPlay) {
      setStatus("Enter a coaching note.");
      return;
    }
    const { tick: anchor, robot, trigger } = editing;
    const note = { tick: anchor, team: editing.team, robot, text };
    if (hasPlay) {
      note.team = playTeam.value;
      if (keys[robot][0] !== note.team) note.robot = lineup[0].robot;
      note.play = { team: note.team, objective, players: lineup };
    }
    notes.set(anchor, note);
    noteOpen(false);
    try {
      localStorage.setItem(storageKey, JSON.stringify([...notes.values()]));
      setStatus(`${hasPlay ? "Play" : "Note"} saved at ${timeText(frames[anchor].t)}.`);
    } catch {
      setStatus("Note kept for this session. Export the moment to save a copy.");
    }
    refreshMarkers();
    seek(anchor);
    trigger.focus();
  });
  el("review-export").addEventListener("click", () => {
    const note = notes.get(tick);
    openClip({
      tick,
      time_s: frames[tick].t,
      team: note ? note.team : team.value || null,
      robot: note?.robot ?? selectedRobot(),
      markers: markers.filter((marker) => marker.tick === tick),
      ...(note?.play && { coaching_play: note.play }),
    }, note?.text ?? "");
  });

  return {
    update(index, ready, source) {
      tick = index;
      toggle.hidden = !ready;
      const reviewing = ready && active;
      toggle.setAttribute("aria-pressed", String(reviewing));
      panel.hidden = track.hidden = !reviewing;
      for (const [button, direction] of [[previous, "Previous"], [next, "Next"]]) {
        const label = `${direction} ${reviewing ? "marked moment" : "role change"}`;
        button.title = label;
        button.setAttribute("aria-label", label);
      }
      if (!reviewing) return;
      if (!markers) {
        markers = indexReviewMarkers(frames, keys, events, robotName, roleName);
        loadNotes(source);
        refreshMarkers();
      }
      showMoment();
      // Role-history rendering also updates these buttons during playback.
      previous.disabled = !ticks.length || tick <= ticks[0];
      next.disabled = !ticks.length || tick >= ticks.at(-1);
    },
    jump(direction) {
      if (!active || !markers) return false;
      const target = direction < 0 ? ticks.findLast((index) => index < tick)
        : ticks.find((index) => index > tick);
      if (target !== undefined) seek(target);
      return true;
    },
  };
}
