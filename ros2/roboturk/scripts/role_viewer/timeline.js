"use strict";

function createTimelineControls(match, currentFrame, currentRobot, timeText) {
  const timeline = match.decision_timeline;
  const onBranch = match.active_timeline === "branch";
  let extend = () => {};

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
    const promote = byId("timeline-promote");
    const status = byId("timeline-status");
    panel.hidden = false;
    original.setAttribute("aria-pressed", String(!onBranch));
    candidate.setAttribute("aria-pressed", String(onBranch));
    byId("candidate-title").textContent = timeline.title;
    candidate.title = timeline.title;
    document.getElementById("events-title").textContent = onBranch ? "Branch events" : "Match events";
    document.getElementById("review-change").hidden = true;
    const forkLabel = `Fork at ${timeText(timeline.fork_time, 2)}`;

    extend = (end) => {
      const scale = Math.max(timeline.parent_end, end, 0.04);
      panel.style.setProperty("--parent-end", `${100 * timeline.parent_end / scale}%`);
      panel.style.setProperty("--fork", `${100 * timeline.fork_time / scale}%`);
      panel.style.setProperty("--branch-end", `${100 * end / scale}%`);
    };

    function renderStatus(latest) {
      if (!match.telemetry_url || latest.status === "finished") extend(latest.trial_end);
      const labels = {
        ready: latest.resumable ? "Webots paused at the end of the trial" : "Recorded open-play trial",
        failed: "Webots trial failed. You can retry.",
        running: "Running Webots trial…", awaiting: "Awaiting a Webots trial",
        switching: "Ending the original match…", continuing: "Live on merged branch", finished: "Merged branch finished",
      };
      status.textContent = `${forkLabel} · ${labels[latest.status]}`;
      byId("main-title").textContent = latest.promoted ? "Original match" : "Main match";
      run.textContent = latest.status === "ready" ? "Run again" : "Run Webots trial";
      run.disabled = ["running", "switching"].includes(latest.status);
      run.hidden = Boolean(latest.promoted);
      promote.hidden = !onBranch || !latest.resumable;
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
        if (["running", "switching", "continuing"].includes(latest.status)) setTimeout(checkTrial, 750);
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
    promote.addEventListener("click", async () => {
      promote.disabled = run.disabled = true;
      status.textContent = "Merging the tested commit and handing off Webots…";
      try {
        const response = await fetch("timeline", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action: "continue" }),
        });
        const value = await response.json();
        if (!response.ok) throw new Error(value.error);
        show(true, timeline.fork_tick);
      } catch (error) {
        status.textContent = error.message || "Could not continue the branch.";
        promote.disabled = run.disabled = false;
      }
    });
    if (["running", "switching", "continuing"].includes(timeline.status)) setTimeout(checkTrial, 750);
  }

  return { open: (frame) => show(true, frame), extend };
}
