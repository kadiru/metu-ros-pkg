// Operate the match table from official match state. TIGERs supplies event evidence.
import { join, dirname } from "node:path";
import { existsSync } from "node:fs";

const [url, ready, firstKickoffTeam, settle = "450", positioning = "reset"] = process.argv.slice(2);
const kickoffDelayMs = Number(settle);
if (!Number.isFinite(kickoffDelayMs) || kickoffDelayMs < 450) {
  throw new Error("Invalid kickoff settling time");
}

const ws = new WebSocket(url);
const send = message => ws.send(JSON.stringify(message));
let state, lastView, since;
let lastSent = 0;
let kickoffs = 0;

ws.onmessage = ({ data }) => {
  const update = JSON.parse(data).matchState;
  if (!update) return;
  if (update.command?.type === "KICKOFF" && (state?.command?.type !== "KICKOFF" || state.command.forTeam !== update.command.forTeam)) kickoffs++;
  state = update;
  const view = JSON.stringify([
    state.stage, state.command, state.nextCommand,
    state.gameEvents?.map(e => [e.type, e.id]),
    state.teamState?.BLUE?.goals, state.teamState?.YELLOW?.goals,
  ]);
  if (lastView !== view) {
    console.log(view);
    lastView = view;
    since = Date.now();
  }
};

ws.onopen = () => {
  const config = { matchType: "GROUP_PHASE", firstKickoffTeam, maxRobotsPerTeam: 5 };
  send({ change: { updateConfigChange: config } });
  console.log("TABLE_READY");
};

function action(match) {
  const { stage, command, nextCommand } = match;
  const expired = parseFloat(match.stageTimeLeft) < 0;
  const events = match.gameEvents ?? [];
  const calls = events.map(event => event.type);
  if (stage === "POST_GAME") return;
  if (calls.includes("POSSIBLE_GOAL") && !calls.some(t => ["GOAL", "INVALID_GOAL"].includes(t))) {
    return;
  }
  if (stage === "NORMAL_HALF_TIME") return expired && ["NEXT_STAGE"];
  if (expired && ["NORMAL_FIRST_HALF", "NORMAL_SECOND_HALF"].includes(stage)) {
    if (command.type !== "HALT") return ["HALT"];
    return [stage === "NORMAL_FIRST_HALF" ? "NEXT_STAGE" : "END_GAME"];
  }
  if (command.type === "HALT") return ["RESUME_FROM_HALT"];
  if (command.type === "KICKOFF") return ["NORMAL_START"];
  if (command.type !== "STOP" || !nextCommand) return;
  const team = nextCommand.forTeam;
  if (nextCommand.type === "KICKOFF") return ["NEXT_COMMAND", team];
  if (nextCommand.type !== "DIRECT") return;
  if (calls.includes("PLACEMENT_SUCCEEDED")) return ["NEXT_COMMAND", team];
  const failures = events.filter(event => event.type === "PLACEMENT_FAILED");
  if (failures.length > 1) return;
  const failed = failures[0]?.placementFailed?.byTeam;
  const otherTeam = failed === "BLUE" ? "YELLOW" : "BLUE";
  return ["BALL_PLACEMENT_START", failed ? otherTeam : team];
}

setInterval(() => {
  const now = Date.now();
  if (!state || !existsSync(ready) || now - since < 450 || now - lastSent < 550) return;
  if (state.stage === "NORMAL_HALF_TIME" && !state.teamState.BLUE.onPositiveHalf) {
    send({ change: { updateTeamStateChange: { forTeam: "BLUE", onPositiveHalf: true } } });
    lastSent = now;
    return;
  }
  if (state.command.type === "KICKOFF" && now - since < kickoffDelayMs) return;
  const decision = action(state);
  if (!decision) return;
  if (decision[0] === "NORMAL_START" && positioning === "drive") {
    const placed = join(dirname(ready), `formation-${kickoffs}-${state.command.forTeam}.ready`);
    if (!existsSync(placed)) return;
  }
  const [type, forTeam = "UNKNOWN"] = decision;
  console.log(JSON.stringify({ stage: state.stage, remaining: state.stageTimeLeft, type, forTeam }));
  send({ continueAction: { type, forTeam } });
  lastSent = now;
}, 100);
