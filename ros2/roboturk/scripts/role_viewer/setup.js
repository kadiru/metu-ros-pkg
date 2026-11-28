"use strict";

const byId = (id) => document.getElementById(id);
const catalog = JSON.parse(byId("match-data").textContent);
const form = byId("setup-form");
const start = byId("start-match");
const teams = ["blue", "yellow"];
const { releases, installed, moments } = catalog.controller_catalog;
const practiceLimits = { ...catalog.limit_defaults };
const motionControls = createLimitControls(byId("motion-limits"), catalog.limit_ranges, (key, value) => {
  practiceLimits[key] = value;
});

for (const team of teams) {
  const controller = byId(`${team}-controller`);
  for (const id of installed) {
    const { name, version } = releases[id];
    controller.add(new Option(`${name} · v${version}`, id));
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
    byId(`${team}-controller-preview`).textContent = releases[controller.value].description;
    byId(`${team}-preview`).textContent = catalog.strategies[strategy.value][1];
  };
  controller.addEventListener("change", describe);
  strategy.addEventListener("change", describe);
  describe();
}

function replayLink(id) {
  const link = document.createElement("a");
  link.href = `/controller-replays/${encodeURIComponent(id)}/`;
  link.textContent = `Replay ${moments[id].name}`;
  return link;
}

for (const release of Object.values(releases)) {
  const article = document.createElement("article");
  const name = document.createElement("h3");
  name.textContent = `${release.name} · v${release.version}`;
  const trigger = document.createElement("p");
  trigger.textContent = release.trigger;
  const changes = document.createElement("ul");
  for (const { description } of release.changes) {
    const item = document.createElement("li");
    item.textContent = description;
    changes.append(item);
  }
  const source = document.createElement("a");
  source.href = `https://github.com/kadiru/metu-ros-pkg/commit/${release.introduced_by}`;
  source.textContent = "Introducing commit";
  article.append(name, trigger, changes, source, ...release.moments.map(replayLink));
  byId("controller-history").append(article);
}
for (const [id, moment] of Object.entries(moments)) {
  const article = document.createElement("article");
  const description = document.createElement("p");
  description.textContent = moment.description;
  article.append(replayLink(id), description);
  byId("controller-moments").append(article);
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
      controller_versions: Object.fromEntries(teams.map(team => [team, byId(`${team}-controller`).value])),
      strategies: Object.fromEntries(teams.map((team) => [team, {
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
byId("setup-offline").hidden = true;
byId("setup-app").hidden = false;
