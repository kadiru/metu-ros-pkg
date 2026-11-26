// Forward fixture operator inputs, leaving event adjudication to TIGERs and the GC.
import readline from "node:readline";
const ws = new WebSocket(process.argv[2]);
await new Promise((resolve, reject) => { ws.onopen = resolve; ws.onerror = reject; });
console.log("READY");
for await (const line of readline.createInterface({input:process.stdin})) {
  ws.send(line);
}
ws.close();
