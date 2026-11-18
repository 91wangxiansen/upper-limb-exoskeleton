// Existing CAD coupling. These angles describe the model, not measured finger joints.
export const MCP_MAX_DEG = 1.03 * 180 / Math.PI;
let hand = null, received = -Infinity;
const target = [0, 0, 0, 0];
export function setHandSnapshot(state, now = performance.now()) {
  hand = state;
  received = now;
}
export function setHandTarget(index, value) {
  if (!Number.isInteger(index) || index < 0 || index > 3 || !Number.isFinite(value)) return;
  target[index] = Math.max(0, Math.min(100, Math.round(value)));
}
export function handView(now = performance.now()) {
  const elapsed = Math.max(0, now - received);
  const live = hand?.connected && Number.isFinite(hand.ageMs) && hand.ageMs + elapsed <= 2000;
  return {target:target.slice(), measured:target.map((_, i) => {
    const f = hand?.fingers?.[i];
    return live && f?.online && typeof f.q === 'number' && Number.isFinite(f.q)
      ? Math.max(0, Math.min(100, f.q)) : null;
  })};
}
