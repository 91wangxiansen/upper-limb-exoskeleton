// SC servo status bits: Waveshare SC Servo Memory Table, section 2.1.
export function alarmText(alarm) {
  if (!alarm) return '';
  const reasons = [];
  if (alarm & 0x01) reasons.push('电压异常（过压或欠压）');
  if (alarm & 0x04) reasons.push('温度过高');
  if (alarm & 0x20) reasons.push('负载过载');
  if (alarm & ~0x25) reasons.push('其他状态位异常');
  return reasons.join('、') + ' · 0x' + alarm.toString(16).padStart(2, '0');
}
