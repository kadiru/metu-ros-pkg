"use strict";

const byId = (id) => document.getElementById(id);
const catalog = JSON.parse(byId("match-data").textContent);
const form = byId("setup-form");
const start = byId("start-match");
const teams = ["blue", "yellow"];
const practiceLimits = { ...catalog.limit_defaults };
const motionControls = createLimitControls(byId("motion-limits"), catalog.limit_ranges, (key, value) => {
  practiceLimits[key] = value;
});

for (const team of teams) {
  const controller = byId(`${team}-controller`);
  for (const [key, [name, description]] of Object.entries(catalog.controllers)) {
    const option = new Option(name, key);
    option.title = description;
    controller.add(option);
  }
  const strategy = byId(`${team}-strategy`);
  for (const [key, [name]] of Object.entries(catalog.strategies)) {
    strategy.add(new Option(name, key));
  }
  const passing = byId(`${team}-passing`);
  for (const preference of [-1, 0, 1]) {
    passing.add(new Option(catalog.passing[preference], preference));
  }
  passing.value = "0";
  const describe = () => {
    byId(`${team}-preview`).textContent = catalog.strategies[strategy.value][1];
  };
  strategy.addEventListener("change", describe);
  describe();
}

function updateMode() {
  const practice = form.elements.mode.value === "practice";
  byId("practice-options").hidden = !practice;
  byId("game-options").hidden = practice;
  const limits = practice ? practiceLimits : catalog.limit_defaults;
  motionControls.show({ blue: limits, yellow: limits }, practice);
  start.textContent = practice ? "Start practice" : "Start game";
}

form.elements.mode.forEach((radio) => radio.addEventListener("change", updateMode));
const periods = form.elements.periods;
const halfSeconds = form.elements.half_seconds;
periods.value = catalog.timing.periods;
[halfSeconds.min, halfSeconds.max] = catalog.half_seconds_range;
halfSeconds.value = catalog.timing.half_seconds;
const updateDuration = () => {
  byId("match-duration").textContent = halfSeconds.validity.valid
    ? `5v5 · ${periods.value} × ${halfSeconds.valueAsNumber} s` : "5v5";
};
periods.addEventListener("change", updateDuration);
halfSeconds.addEventListener("input", updateDuration);
updateDuration();

form.addEventListener("submit", async (event) => {
  event.preventDefault();
  start.disabled = true;
  byId("setup-error").textContent = "";
  byId("active-match").hidden = true;
  const mode = form.elements.mode.value;
  const request = mode === "game"
    ? { mode, team: form.elements.team.value }
    : {
      mode,
      strategies: Object.fromEntries(teams.map((team) => [team, {
        controller: byId(`${team}-controller`).value,
        strategy: byId(`${team}-strategy`).value,
        pass_preference: Number(byId(`${team}-passing`).value),
      }])),
      limits: practiceLimits,
    };
  request.periods = Number(periods.value);
  request.half_seconds = halfSeconds.valueAsNumber;
  try {
    const response = await fetch("/matches", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request),
    });
    const result = await response.json();
    if (response.ok) {
      window.location.assign(result.url);
      return;
    }
    if (response.status === 409) {
      const active = byId("active-match");
      active.href = result.url;
      active.hidden = false;
    }
    byId("setup-error").textContent = result.error;
  } catch {
    byId("setup-error").textContent = "The local viewer could not start the match.";
  }
  start.disabled = false;
});

updateMode();
