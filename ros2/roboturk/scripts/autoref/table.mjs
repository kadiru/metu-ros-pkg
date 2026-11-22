// Drive the referee-table Continue buttons in the simulation fixture.
// The AutoRef and Game Controller alone propose, validate and credit the goal.
const socket = new WebSocket(process.argv[2]);
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
let previous = "";
let lastAction = "";
let started = false;
socket.onmessage = ({data}) => {
  const state = JSON.parse(data).matchState;
  if (!state) return;
  const command = state.command?.type;
  const next = state.nextCommand;
  const goals = ["BLUE", "YELLOW"].map(t => state.teamState?.[t]?.goals ?? 0);
  const view = {command, next, goals, events:state.gameEvents?.map(e => e.type)};
  if (JSON.stringify(view) !== previous) {
    console.log(JSON.stringify(view));
    previous = JSON.stringify(view);
  }
  if (!started || !goals.some(g => g > 0)) return;
  let type;
  if (command === "HALT" && next?.type === "KICKOFF") type = "RESUME_FROM_HALT";
  if (command === "STOP" && next?.type === "KICKOFF") type = "NEXT_COMMAND";
  if (command === "KICKOFF") type = "NORMAL_START";
  if (type && type !== lastAction) {
    lastAction = type;
    setTimeout(() => {
      if (socket.readyState === WebSocket.OPEN) {
        console.log("Continue", type);
        socket.send(JSON.stringify({continueAction:{type,forTeam:"UNKNOWN"}}));
      }
    }, 1000);
  }
};
await sleep(150);
socket.send(JSON.stringify({change:{newCommandChange:{command:{type:"FORCE_START",forTeam:"UNKNOWN"}}}}));
started = true;
console.log("TABLE_READY");
