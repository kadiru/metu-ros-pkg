"use strict";

function createTimelineControls(match, currentFrame, currentRobot, timeText) {
  const timeline = match.decision_timeline;
  const onBranch = match.active_timeline === "branch";

  function show(branch, frame) {
    const url = new URL(location.href);
    if (branch) url.searchParams.set("timeline", "branch");
    else url.searchParams.delete("timeline");
    url.searchParams.set("frame", frame);
    const robot = currentRobot();
    if (robot) url.searchParams.set("robot", robot);
    location.assign(url);
  }

  if (timeline) {
    const byId = (id) => document.getElementById(id);
    const panel = byId("decision-timelines");
    const original = byId("main-timeline");
    const candidate = byId("candidate-timeline");
    const run = byId("timeline-run");
    const status = byId("timeline-status");
    panel.hidden = false;
    original.setAttribute("aria-pressed", String(!onBranch));
    candidate.setAttribute("aria-pressed", String(onBranch));
    byId("candidate-title").textContent = timeline.title;
    candidate.title = timeline.title;
    document.getElementById("events-title").textContent = onBranch ? "Branch events" : "Match events";
    document.getElementById("review-change").hidden = true;
    const forkLabel = `Fork at ${timeText(timeline.fork_time, 2)}`;

    function renderStatus(latest) {
      const scale = Math.max(timeline.parent_end, latest.trial_end, 0.04);
      panel.style.setProperty("--parent-end", `${100 * timeline.parent_end / scale}%`);
      panel.style.setProperty("--fork", `${100 * timeline.fork_time / scale}%`);
      panel.style.setProperty("--branch-end", `${100 * latest.trial_end / scale}%`);
      const labels = {
        ready: "Reconstructed open-play trial", failed: "Webots trial failed. You can retry.",
        running: "Running Webots trial…", awaiting: "Awaiting a Webots trial",
      };
      status.textContent = `${forkLabel} · ${labels[latest.status]}`;
      run.textContent = latest.status === "ready" ? "Run again" : "Run Webots trial";
      run.disabled = latest.status === "running";
    }

    renderStatus(timeline);
    original.addEventListener("click", () => {
      if (onBranch) show(false, Math.min(currentFrame(), timeline.parent_last));
    });
    candidate.addEventListener("click", () => {
      if (!onBranch) show(true, timeline.fork_tick);
    });
    byId("timeline-reset").addEventListener("click", () => show(false, timeline.fork_tick));

    async function checkTrial() {
      try {
        const response = await fetch("timeline");
        if (!response.ok) throw new Error();
        const { timeline: latest } = await response.json();
        if (!latest) throw new Error();
        if (latest.status === "running") setTimeout(checkTrial, 750);
        else if (onBranch && latest.status === "ready") show(true, timeline.fork_tick);
        else renderStatus(latest);
      } catch {
        status.textContent = "Could not check the Webots trial. Reload Hoca to reconnect.";
      }
    }

    run.addEventListener("click", async () => {
      run.disabled = true;
      try {
        const response = await fetch("timeline", {
          method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
        });
        const value = await response.json();
        if (!response.ok) throw new Error(value.error);
        if (onBranch) {
          renderStatus(value);
          setTimeout(checkTrial, 750);
        } else show(true, timeline.fork_tick);
      } catch (error) {
        status.textContent = error.message || "Could not start the Webots trial.";
        run.disabled = false;
      }
    });
    if (timeline.status === "running") setTimeout(checkTrial, 750);
  }

  return { open: (frame) => show(true, frame) };
}
