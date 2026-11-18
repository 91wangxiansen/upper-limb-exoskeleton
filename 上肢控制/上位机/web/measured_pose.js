// Display-only telemetry policy. Targets and control phases never drive the model.
const finite = value => typeof value === 'number' && Number.isFinite(value);
const keys = ['shoulder', 'elbow'];
export const POSE_FRESHNESS_S = .3;

export function measuredPoseView(state, elapsedSinceReceiveS = 0, serviceAvailable = true) {
  const elapsed = finite(elapsedSinceReceiveS) && elapsedSinceReceiveS >= 0 ? elapsedSinceReceiveS : Infinity;
  const connected = serviceAvailable && state?.connected === true;
  const joints = Object.fromEntries(keys.map(key => {
    const joint = state?.joints?.[key];
    const known = joint?.verified === true && finite(joint.actual_deg) &&
      finite(joint.feedback_age_s) && joint.feedback_age_s >= 0;
    const age = known ? joint.feedback_age_s + elapsed : null;
    return [key, {angle: known ? joint.actual_deg : null, age,
      fresh: connected && known && age <= POSE_FRESHNESS_S}];
  }));
  const fresh = keys.every(key => joints[key].fresh);
  const known = keys.every(key => joints[key].angle !== null);
  const status = !serviceAvailable ? 'unavailable' : !state?.connected ? 'offline' : !known ? 'waiting' : fresh ? 'live' : 'stale';
  const text = {unavailable:'后台连接中断 · 非实时', offline:'未连接 · 非实时', waiting:'等待双关节位置反馈',
    live:'实时实际姿态', stale:'反馈过期 · 最后已知姿态'}[status];
  return {joints, fresh, known, connected, status, text};
}
