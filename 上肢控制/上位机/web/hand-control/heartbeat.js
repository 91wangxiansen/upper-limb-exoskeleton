// Keep the controller lease alive independently of status polling and UI actions.
// Never overlap heartbeat requests or replay motion after a connection failure.
export function startHeartbeat({send,isActive,onError=()=>{},interval=600,schedule=setInterval,cancel=clearInterval}) {
  let pending=false,stopped=false;
  const tick=async()=>{
    if(stopped||pending||!isActive())return;
    pending=true;
    try{await send();}catch(error){onError(error);}finally{pending=false;}
  };
  const timer=schedule(tick,interval);
  return ()=>{stopped=true;cancel(timer);};
}
