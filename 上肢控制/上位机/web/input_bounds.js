const installed = new WeakSet();
export function setRange(input, minimum, maximum, integer=false) {
  for (const [key,value] of [['min',minimum],['max',maximum]]) {
    if (Number.isFinite(value)) input[key]=String(value);
    else input.removeAttribute(key);
  }
  if (integer) input.step='1';
  else if (input.type === 'number') input.step='any';
  input.dataset.integer = String(integer);
  if (input.value!=='' && Number.isFinite(Number(input.value))) {
    let value=Number(input.value);
    if (Number.isFinite(minimum)) value=Math.max(minimum,value);
    if (Number.isFinite(maximum)) value=Math.min(maximum,value);
    if (allowed(input,value)) input.value=String(value);
  }
  const left = Number.isFinite(minimum) ? minimum : '无固定下限';
  const right = Number.isFinite(maximum) ? maximum : '无固定上限';
  const relation = input.dataset.exclusiveMax ? '，须小于上界' : input.dataset.exclusiveMin ? '，须大于下界' : '';
  input.title = `允许范围 ${left} ～ ${right}${integer ? '，整数' : ''}${relation}`;
  input.setAttribute('aria-description',input.title);
}
function allowed(input, value) {
  return Number.isFinite(value) && (input.min === '' || (input.dataset.exclusiveMin ? value>Number(input.min) : value>=Number(input.min))) &&
    (input.max === '' || (input.dataset.exclusiveMax ? value<Number(input.max) : value<=Number(input.max)));
}
export function bindNumericBounds(root) {
  if (installed.has(root)) return;
  installed.add(root);
  const numeric = event => event.target?.matches?.('input[type="number"],input[type="range"]') ? event.target : null;
  root.addEventListener('focusin',event=>{
    const input=numeric(event);
    if (input && input.value!=='' && allowed(input,Number(input.value))) input.dataset.lastValid=input.value;
  });
  root.addEventListener('input',event=>{
    const input=numeric(event);
    if (!input) return;
    input.setCustomValidity('');
    if (input.value==='' || !Number.isFinite(Number(input.value))) {
      input.setCustomValidity('请输入允许范围内的数值');
      return;
    }
    let value=Number(input.value);
    if (input.dataset.integer==='true') value=Math.round(value);
    if (input.min!=='') value=Math.max(Number(input.min),value);
    if (input.max!=='') value=Math.min(Number(input.max),value);
    if (!allowed(input,value)) {
      const previous=Number(input.dataset.lastValid);
      if (input.dataset.lastValid!==undefined && allowed(input,previous)) value=previous;
      else { input.setCustomValidity(input.title); return; }
    }
    input.value=String(value);
    input.dataset.lastValid=input.value;
  },true);
  root.addEventListener('change',event=>{
    const input=numeric(event);
    if (input && input.value==='' && input.dataset.lastValid!==undefined) {
      input.value=input.dataset.lastValid;
      input.dispatchEvent(new Event('input',{bubbles:true}));
    }
  },true);
}
export function validInputs(inputs) {
  return [...inputs].every(input=>{
    input.setCustomValidity(input.value!=='' && allowed(input,Number(input.value)) ? '' : '请输入允许范围内的数值');
    return input.reportValidity();
  });
}
