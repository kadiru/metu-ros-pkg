"use strict";

const byId = (id) => document.getElementById(id);
const catalog = JSON.parse(byId("match-data").textContent);
const form = byId("setup-form");
const start = byId("start-match");
const teams = ["blue", "yellow"];
const limitFields = {
  max_speed: ["Top speed", "m/s"],
  max_omega: ["Top rotation", "rad/s"],
  max_acceleration: ["Acceleration / braking", "m/s²"],
  max_kick: ["Kick speed", "m/s"],
};

for (const team of teams) {
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
  const limits = byId(`${team}-limits`);
  for (const [key, [title, unit]] of Object.entries(limitFields)) {
    const label = document.createElement("label");
    const caption = document.createElement("span");
    caption.textContent = title;
    const control = document.createElement("span");
    const input = document.createElement("input");
    const valueUnit = document.createElement("span");
    valueUnit.textContent = unit;
    input.type = "number";
    input.id = `${team}-${key}`;
    input.value = catalog.limit_defaults[key];
    [input.min, input.max] = catalog.limit_ranges[key];
    input.step = "any";
    input.required = true;
    input.setAttribute("aria-label", `${team[0].toUpperCase() + team.slice(1)} ${title}`);
    control.append(input, valueUnit);
    label.append(caption, control);
    limits.append(label);
  }
}

function updateMode() {
  const practice = form.elements.mode.value === "practice";
  byId("practice-options").hidden = !practice;
  byId("game-options").hidden = practice;
  for (const input of byId("practice-options").querySelectorAll("input")) input.disabled = !practice;
  start.textContent = practice ? "Start practice" : "Start game";
}

form.elements.mode.forEach((radio) => radio.addEventListener("change", updateMode));
byId("match-duration").textContent = `5v5 · 2 × ${catalog.half_seconds} s`;
byId("game-limit-summary").textContent = Object.entries(limitFields).map(([key, [title, unit]]) =>
  `${title} ${catalog.limit_defaults[key]} ${unit}`).join(" · ");

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
        strategy: byId(`${team}-strategy`).value,
        pass_preference: Number(byId(`${team}-passing`).value),
      }])),
      limits: Object.fromEntries(teams.map((team) => [team, Object.fromEntries(
        Object.keys(limitFields).map((key) => [key, byId(`${team}-${key}`).valueAsNumber]),
      )])),
    };
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
