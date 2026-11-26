// Act on the referee's awarded follow-up. All proposals originate in TIGERs.
const socket = new WebSocket(process.argv[2]);
await new Promise((resolve,reject) => { socket.onopen=resolve; socket.onerror=reject; });
let last;
socket.onmessage = ({data}) => {
  const state=JSON.parse(data).matchState;
  if (!state || state.command?.type!=="STOP" || state.nextCommand?.type!=="DIRECT") return;
  const event=state.gameEvents?.at(-1);
  const type=event?.type==="PLACEMENT_SUCCEEDED" ? "NEXT_COMMAND" :
    event?.type==="BALL_LEFT_FIELD_TOUCH_LINE" ? "BALL_PLACEMENT_START" : null;
  if (!type || event.id===last) return;
  last=event.id;
  socket.send(JSON.stringify({continueAction:{type,forTeam:state.nextCommand.forTeam}}));
};
socket.send(JSON.stringify({change:{newCommandChange:{command:{type:"FORCE_START",forTeam:"UNKNOWN"}}}}));
console.log("TABLE_READY");
