// Operate the match table from official match state. TIGERs supplies event evidence.
import {existsSync} from "node:fs";

const [url, ready, firstKickoffTeam] = process.argv.slice(2);
const ws = new WebSocket(url);
let state, lastView, since, lastSent = 0;
const send = message => ws.send(JSON.stringify(message));
ws.onmessage = ({data}) => {
  const update = JSON.parse(data).matchState;
  if (!update) return;
  state = update;
  const view = JSON.stringify([state.stage, state.command, state.nextCommand,
    state.gameEvents?.map(e => [e.type, e.id]), state.teamState?.BLUE?.goals, state.teamState?.YELLOW?.goals]);
  if (lastView !== view) {
    console.log(view);
    lastView = view;
    since = Date.now();
  }
};
ws.onopen = () => {
  send({change:{updateConfigChange:{matchType:"GROUP_PHASE", firstKickoffTeam, maxRobotsPerTeam:5}}});
  console.log("TABLE_READY");
};

function action(s) {
  const {stage, command, nextCommand} = s;
  const remaining = parseFloat(s.stageTimeLeft);
  const calls = (s.gameEvents ?? []).map(e => e.type);
  if (stage === "POST_GAME") return;
  if (calls.includes("POSSIBLE_GOAL") && !calls.some(t => ["GOAL","INVALID_GOAL"].includes(t))) return;
  if (stage === "NORMAL_HALF_TIME") return remaining < 0 && ["NEXT_STAGE"];
  if (["NORMAL_FIRST_HALF","NORMAL_SECOND_HALF"].includes(stage) && remaining < 0) {
    if (command.type !== "HALT") return ["HALT"];
    return [stage === "NORMAL_FIRST_HALF" ? "NEXT_STAGE" : "END_GAME"];
  }
  if (command.type === "HALT") return ["RESUME_FROM_HALT"];
  if (command.type === "KICKOFF") return ["NORMAL_START"];
  if (command.type !== "STOP" || !nextCommand) return;
  const team = nextCommand.forTeam;
  if (nextCommand.type === "KICKOFF") return ["NEXT_COMMAND", team];
  if (nextCommand.type === "DIRECT") {
    if (calls.includes("PLACEMENT_SUCCEEDED")) return ["NEXT_COMMAND", team];
    const failures = (s.gameEvents ?? []).filter(e => e.type === "PLACEMENT_FAILED");
    if (failures.length > 1) return;
    const failed = failures[0]?.placementFailed?.byTeam;
    return ["BALL_PLACEMENT_START", failed ? (failed === "BLUE" ? "YELLOW" : "BLUE") : team];
  }
}
setInterval(() => {
  if (!state || !existsSync(ready) || Date.now() - since < 450 || Date.now() - lastSent < 550) return;
  if (state.stage === "NORMAL_HALF_TIME" && !state.teamState.BLUE.onPositiveHalf) {
    send({change:{updateTeamStateChange:{forTeam:"BLUE", onPositiveHalf:true}}});
    lastSent = Date.now();
    return;
  }
  const decision = action(state);
  if (!decision) return;
  const [type, forTeam = "UNKNOWN"] = decision;
  console.log(JSON.stringify({stage:state.stage, remaining:state.stageTimeLeft, type, forTeam}));
  send({continueAction:{type, forTeam}});
  lastSent = Date.now();
}, 100);
