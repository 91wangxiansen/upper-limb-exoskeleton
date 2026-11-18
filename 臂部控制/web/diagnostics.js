// Read-only diagnostic presentation. Never sends commands or changes limits.
const finite = value => typeof value === 'number' && Number.isFinite(value);
const ageWithin = (age, limit) => finite(age) && age >= 0 && age <= limit;
const number = (value, digits = 2) => finite(value) ? value.toFixed(digits) : '—';
const phases = {stopped:'已停止', moving:'轨迹运动中', controlling:'持续位置控制', settling:'确认到位', holding:'到位保持', recovering:'扰动恢复', limited:'尚未到位'};

export function signedMeter(value, limit) {
  if (!finite(value) || !finite(limit) || limit <= 0) return {left:50, width:0, valid:false, overflow:false};
  const width = Math.min(Math.abs(value) / limit, 1) * 50;
  return {left: value < 0 ? 50 - width : 50, width, valid:true, overflow:Math.abs(value) > limit};
}

export function diagnosticView(joint, state) {
  const connected = state.connected === true;
  const feedbackFresh = connected && ageWithin(joint.feedback_age_s, .3);
  const fault = !!state.safety_latched || !!joint.fault_bits;
  const sent = joint.sent_control;
  const commandFresh = feedbackFresh && !fault && state.motion_active === true &&
    ['moving','controlling','settling','holding','recovering','limited'].includes(joint.phase) && !!sent && ageWithin(sent.age_s, .3);
  const feedback = key => feedbackFresh && finite(joint[key]) ? joint[key] : null;
  const command = key => commandFresh && finite(sent[key]) ? sent[key] : null;
  const parameter = key => {
    const item = joint.diagnostics?.[key];
    return connected && item?.fresh === true && ageWithin(item.age_s, 3) && finite(item.value) ? item.value : null;
  };
  const sentLimit = command('torque_limit_nm');
  const readbackLimit = parameter('limit_torque_nm');
  const limit = sentLimit > 0 ? sentLimit : readbackLimit > 0 ? readbackLimit : null;
  const estimate = command('estimated_total_nm');
  const nearLimit = finite(estimate) && sentLimit > 0 && Math.abs(estimate) >= .95 * sentLimit;
  const mode = parameter('run_mode');
  return {
    phase: !connected ? '离线' : fault ? '安全停止' : !feedbackFresh ? '反馈过期' : joint.phase === 'controlling' ? `持续控制 · ${joint.on_target === true ? '已到位' : '有偏差 / 未稳定'}` : (phases[joint.phase] || '状态未知'),
    phaseTone: !connected ? 'muted' : fault ? 'danger' : !feedbackFresh || ['limited','recovering'].includes(joint.phase) ? 'warn' : 'ok',
    feedbackText: feedbackFresh ? `反馈 ${number(joint.feedback_age_s, 2)} s` : connected ? '反馈过期 / 缺失' : '未连接',
    feedbackTone: feedbackFresh ? 'ok' : connected ? 'warn' : 'muted',
    mode: ['运控模式','位置模式','速度模式','电流模式'][mode] || '模式未读到',
    modeTone: finite(mode) && mode === 0 ? 'ok' : finite(mode) ? 'warn' : 'muted',
    error: feedback('error_deg'), iq: parameter('iq_a'), voltage: parameter('vbus_v'),
    limit, limitText: `${sentLimit > 0 ? '本次上限' : readbackLimit > 0 ? '上限读回' : '量程未知'}${limit ? ` ±${number(limit)} Nm` : ''}`,
    commandText: nearLimit ? '估算接近上限' : commandFresh ? '下发数据新鲜' : '无新鲜下发',
    commandTone: nearLimit ? 'warn' : 'muted',
    rows: [
      {key:'actual', label:'反馈力矩', tag:'实测', value:feedback('torque_nm')},
      {key:'estimate', label:'PD + 补偿', tag:'估算', value:estimate},
      {key:'feedforward', label:'前馈字段', tag:'已发送', value:command('feedforward_nm')},
    ].map(row => ({...row, ...signedMeter(row.value, limit)})),
    details: {
      gravity: commandFresh && finite(joint.gravity_torque_nm) ? joint.gravity_torque_nm : null,
      bias: commandFresh && finite(joint.load_bias_nm) ? joint.load_bias_nm : null,
      pd:command('pd_estimate_nm'), kp:command('kp'), kd:command('kd'),
      readback:readbackLimit, currentLimit:parameter('limit_cur_a'),
    },
  };
}

export function mountDiagnostics(container) {
  container.className = 'joint-diagnostics';
  container.innerHTML = `
    <div class="diagnostic-status"><span data-diagnostic="phase" class="status-badge"></span><span data-diagnostic="mode" class="status-badge"></span><span data-diagnostic="freshness" class="freshness"></span></div>
    <div class="diagnostic-metrics">
      <div><span>目标误差</span><b data-diagnostic="error">—</b><small>°</small></div>
      <div><span>实测 Iq</span><b data-diagnostic="iq">—</b><small>A</small></div>
      <div><span>母线电压</span><b data-diagnostic="voltage">—</b><small>V</small></div>
    </div>
    <div class="effort-heading"><b>力矩对照 <small>Nm · 电机端</small></b><span data-diagnostic="limit"></span></div>
    <div class="effort-rows">${[
      ['actual','反馈力矩','实测'], ['estimate','PD + 补偿','估算'], ['feedforward','前馈字段','已发送']
    ].map(([key,label,tag]) => `<div class="effort-row" data-effort="${key}"><div class="effort-label"><span>${label}</span><em>${tag}</em><b data-value>—</b></div><div class="effort-track" role="img" aria-label="${label}暂无数据"><i data-bar></i><span class="effort-zero"></span></div></div>`).join('')}</div>
    <div class="effort-axis"><span data-diagnostic="axisMin">—</span><span>0</span><span data-diagnostic="axisMax">—</span></div>
    <div class="effort-note"><span data-diagnostic="command" class="status-badge"></span><span>估算 ≠ 实测 · 发送 ≠ 执行</span></div>
    <details class="diagnostic-details"><summary>出力分项与参数</summary>
      <div class="diagnostic-detail-grid">${[
        ['gravity','重力前馈','Nm'], ['bias','负载误差补偿','Nm'], ['pd','PD 估算','Nm'],
        ['kp','已下发 Kp',''], ['kd','已下发 Kd',''], ['readback','总转矩上限读回','Nm'],
        ['currentLimit','位置/速度模式电流上限','A'],
      ].map(([key,label,unit]) => `<div><span>${label}</span><b data-detail="${key}">—</b><small>${unit}</small></div>`).join('')}</div>
      <p class="diagnostic-caption">电流上限参数不代表当前运控总转矩上限；过期或缺失数据以 — 显示。</p>
    </details>`;
  const fields = Object.fromEntries([...container.querySelectorAll('[data-diagnostic]')].map(el => [el.dataset.diagnostic, el]));
  const details = Object.fromEntries([...container.querySelectorAll('[data-detail]')].map(el => [el.dataset.detail, el]));
  const rows = Object.fromEntries([...container.querySelectorAll('[data-effort]')].map(el => [el.dataset.effort,
    {value:el.querySelector('[data-value]'), bar:el.querySelector('[data-bar]'), track:el.querySelector('.effort-track')} ]));
  return (joint, state) => {
    const view = diagnosticView(joint, state);
    for (const [key,text,tone] of [['phase',view.phase,view.phaseTone], ['mode',view.mode,view.modeTone],
      ['freshness',view.feedbackText,view.feedbackTone], ['command',view.commandText,view.commandTone]]) {
      fields[key].textContent = text;
      fields[key].dataset.tone = tone;
    }
    for (const key of ['error','iq','voltage']) fields[key].textContent = number(view[key]);
    fields.limit.textContent = view.limitText;
    fields.axisMin.textContent = view.limit ? `−${number(view.limit)}` : '—';
    fields.axisMax.textContent = view.limit ? `+${number(view.limit)}` : '—';
    for (const row of view.rows) {
      const elements = rows[row.key];
      elements.value.textContent = `${number(row.value, 3)}${row.overflow ? ' · 超量程' : ''}`;
      elements.bar.style.left = `${row.left}%`;
      elements.bar.style.width = `${row.width}%`;
      elements.track.dataset.missing = String(!row.valid);
      elements.track.setAttribute('aria-label', `${row.label}（${row.tag}）：${finite(row.value) ? `${number(row.value,3)} Nm${row.overflow ? '，超出显示量程' : ''}` : '暂无新鲜数据'}`);
    }
    for (const [key,value] of Object.entries(view.details)) details[key].textContent = number(value, key === 'kp' || key === 'kd' ? 3 : 2);
  };
}
