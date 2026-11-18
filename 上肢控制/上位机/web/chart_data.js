// Display processing only. Raw feedback stays separate from these plotted values.
export const WINDOW_SECONDS = 60;
export const BIN_SECONDS = .1;
export const FILTER_TAU_SECONDS = .35;
const GAP_SECONDS = 1;
const JOINTS = ['shoulder','elbow'];
const METRICS = ['angle','velocity','torque','temperature'];
const MINIMUM_SPAN = {angle:1,velocity:.4,torque:.2,temperature:1};
const validPoint = row => Array.isArray(row) && Number.isFinite(row[0]) && Number.isFinite(row[1]);

export function mergeHistory(previous, incoming, now) {
  return Object.fromEntries(JOINTS.map(joint=>[joint,Object.fromEntries(METRICS.map(metric=>{
    const supplied = incoming[joint]?.[metric];
    if (!Array.isArray(supplied) || !supplied.length) return [metric,[]];
    const points = new Map();
    for (const row of [...(previous[joint]?.[metric] || []),...supplied]) {
      if (Array.isArray(row) && Number.isFinite(row[0]) && row[0]>=now-WINDOW_SECONDS && row[0]<=now) {
        points.set(row[0],Number.isFinite(row[1]) ? row[1] : null);
      }
    }
    return [metric,[...points].sort((a,b)=>a[0]-b[0])];
  }))]));
}

export function smoothSeries(rows) {
  const segments=[];
  let segment=[], bucket=null, previousTime=null, filtered=null, filteredAt=null;
  function flush() {
    if (!bucket) return;
    const mean=bucket.sum/bucket.count;
    const alpha=filtered===null ? 1 : 1-Math.exp(-(bucket.time-filteredAt)/FILTER_TAU_SECONDS);
    filtered=filtered===null ? mean : filtered+alpha*(mean-filtered);
    filteredAt=bucket.time;
    segment.push([bucket.time,filtered]);
    bucket=null;
  }
  function finish() {
    flush();
    if (segment.length) segments.push(segment);
    segment=[];filtered=null;filteredAt=null;previousTime=null;
  }
  for (const row of rows) {
    if (!validPoint(row)) { finish(); continue; }
    const [time,value]=row;
    if (previousTime!==null && (time-previousTime>GAP_SECONDS || time<=previousTime)) finish();
    const id=Math.floor(time/BIN_SECONDS);
    if (bucket && bucket.id!==id) flush();
    if (!bucket) bucket={id,time,sum:0,count:0};
    bucket.sum+=value;bucket.count++;bucket.time=time;
    previousTime=time;
  }
  finish();
  return segments;
}

export function chartView(history, metric, now) {
  const series=Object.fromEntries(JOINTS.map(joint=>[joint,smoothSeries(
    (history[joint]?.[metric] || []).filter(row=>row[0]>=now-WINDOW_SECONDS && row[0]<=now)
  )]));
  const points=JOINTS.flatMap(joint=>series[joint].flat());
  if (!points.length) return {series,empty:true};
  const times=points.map(p=>p[0]), values=points.map(p=>p[1]);
  const end=Math.max(...times), first=Math.min(...times), span=Math.max(1,end-first);
  let low=Math.min(...values), high=Math.max(...values);
  const width=Math.max(high-low,MINIMUM_SPAN[metric] || .01), center=(low+high)/2;
  low=center-width/2;high=center+width/2;
  const padding=width*.12;
  low-=padding;high+=padding;
  return {series,empty:false,start:end-span,end,span,low,high,stale:now-end>2};
}
