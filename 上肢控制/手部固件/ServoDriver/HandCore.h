#pragma once
// Hardware-independent control state machine. All UART access has one owner:
// control task. Web handlers use the same mutex; no motion loops in handlers.
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include "HandContact.h"

namespace hand {
constexpr int N = 4;
constexpr uint32_t FRESH_MS = 240, GRIP_TIMEOUT_MS = 20000;
constexpr uint32_t STEP_MS = 40, MAX_STEP_INTERVAL_MS = 80;
constexpr int MAX_MOVE_STEP = 40, MAX_GRIP_STEP = 12;
struct Config {
  uint8_t id = 0, verified = 0;
  int16_t open = 500, close = 1000;
  uint16_t speed = 100, loadLimit = 300;
};
inline Config defaults(int i) {
  const int16_t opens[N] = {250,150,800,600}, closes[N] = {600,650,300,200};
  Config c; c.id = uint8_t(1+i); c.open = opens[i]; c.close = closes[i]; return c;
}
inline bool validConfig(const Config (&c)[N]) {
  for(int i=0;i<N;++i) {
    if(c[i].id<1 || c[i].id>20 || c[i].open<0 || c[i].open>1023 ||
       c[i].close<0 || c[i].close>1023 || abs(c[i].close-c[i].open)<20 ||
       c[i].speed<20 || c[i].speed>500 || c[i].loadLimit<50 || c[i].loadLimit>1000 || c[i].verified>1) return false;
    for(int j=0;j<i;++j) if(c[i].id==c[j].id) return false;
  }
  return true;
}
inline float clamp01(float x) { return std::max(0.0f,std::min(1.0f,x)); }
inline int raw(const Config& c,float q) { return int(lroundf(c.open+clamp01(q)*(c.close-c.open))); }
inline float closure(const Config& c,int p) { return clamp01(float(p-c.open)/float(c.close-c.open)); }
enum class Phase : uint8_t { Idle, Moving, Gripping, Contact, Limit, Held, Fault };
inline const char* phaseName(Phase p) {
  const char* names[] = {"idle","moving","gripping","contact","limit","held","fault"};
  return names[int(p)];
}
struct Sample {
  int position = -1, load = 0, speed = 0, voltage = 0, temperature = 0;
  int mode = -1;
  int alarm = 0;
  // 0 = complete read, 1 = feedback packet unavailable, 2 = mode read failed.
  int readError = 0;
};
struct State {
  Sample fb;
  uint32_t stamp = 0, lastStep = 0;
  bool seen = false, online = false, enabled = false;
  uint8_t failures = 0, hits = 0;
  int readError = 0, alarm = 0;
  float target = 0;
  Phase phase = Phase::Idle;
  ContactDetector contact;
};
struct Bus {
  virtual ~Bus() {}
  virtual bool read(uint8_t id,Sample& out) = 0;
  virtual bool move(uint8_t id,int position,int speed) = 0;
  virtual bool torque(uint8_t id,bool on) = 0;
};
class Controller {
 public:
  Config config[N]; State state[N];
  bool armed = false, gripping = false;
  const char* fault = "";
  uint32_t heartbeat = 0, gripStarted = 0;
  explicit Controller(Bus& b):bus(b) { for(int i=0;i<N;++i) config[i]=defaults(i); }
  bool fresh(int i,uint32_t now) const { return state[i].seen && state[i].online && uint32_t(now-state[i].stamp)<=FRESH_MS; }
  bool moving() const {
    for(const auto& s:state) if(s.phase==Phase::Moving || s.phase==Phase::Gripping) return true;
    return false;
  }
  bool ready(uint32_t now) const {
    if(!validConfig(config)) return false;
    for(int i=0;i<N;++i) {
      const auto& s=state[i]; const auto& c=config[i];
      if(!c.verified || !fresh(i,now) || s.fb.mode!=0 || s.alarm || s.fb.temperature>=65 || s.fb.position<std::min(c.open,c.close)-12 ||
         s.fb.position>std::max(c.open,c.close)+12 || abs(s.fb.load)>=c.loadLimit) return false;
    }
    return true;
  }
  void beat(uint32_t now) { heartbeat=now; }
  // Preload measured targets while torque is OFF, then enable. Never restore an
  // EEPROM-era target, and never command "home" automatically on boot/reconnect.
  bool arm(uint32_t now) {
    if(armed || !ready(now)) return false;
    for(int i=0;i<N;++i) if(!bus.torque(config[i].id,false)) { release("enable_failed"); return false; }
    for(int i=0;i<N;++i) {
      state[i].target=closure(config[i],state[i].fb.position);
      if(!bus.move(config[i].id,state[i].fb.position,config[i].speed)) { release("enable_failed"); return false; }
    }
    for(int i=0;i<N;++i) {
      if(!bus.torque(config[i].id,true)) { release("enable_failed"); return false; }
      state[i].enabled=true; state[i].phase=Phase::Held; state[i].hits=0;
      state[i].contact.reset(now,state[i].fb.position,0);
    }
    heartbeat=now; armed=true; fault=""; return true;
  }
  void release(const char* reason="") {
    // Always send torque-off, including when a write acknowledgement was lost.
    for(int i=0;i<N;++i) {
      bus.torque(config[i].id,false);
      state[i].enabled=false; state[i].phase=reason[0]?Phase::Fault:Phase::Idle;
    }
    armed=false; gripping=false; fault=reason;
  }
  void hold(uint32_t now) {
    gripping=false;
    if(!armed) return;
    for(int i=0;i<N;++i) {
      if(!fresh(i,now) || !bus.move(config[i].id,state[i].fb.position,config[i].speed)) { release("feedback_lost"); return; }
      state[i].target=closure(config[i],state[i].fb.position); state[i].phase=Phase::Held;
      state[i].contact.reset(now,state[i].fb.position,0);
    }
  }
  bool targets(const float (&q)[N],uint32_t now) {
    if(!armed || gripping) return false;
    for(int i=0;i<N;++i) if(!fresh(i,now) || !std::isfinite(q[i]) || q[i]<0 || q[i]>1) return false;
    for(int i=0;i<N;++i) {
      auto& s=state[i]; const int delta=raw(config[i],q[i])-s.fb.position;
      const int dir=(delta>0)-(delta<0);
      // Repeated live targets in the same direction must not erase evidence.
      if(s.phase!=Phase::Moving || dir!=s.contact.direction) s.contact.reset(now,s.fb.position,dir);
      s.target=q[i]; s.phase=Phase::Moving; s.hits=0; s.lastStep=now-STEP_MS;
    }
    return true;
  }
  bool grip(uint32_t now) {
    if(!armed || gripping) return false;
    for(int i=0;i<N;++i) if(!fresh(i,now)) return false;
    for(int i=0;i<N;++i) {
      state[i].target=1; state[i].phase=Phase::Gripping; state[i].hits=0; state[i].lastStep=now-STEP_MS;
      const int delta=config[i].close-state[i].fb.position;
      state[i].contact.reset(now,state[i].fb.position,(delta>0)-(delta<0));
    }
    gripping=true; gripStarted=now; return true;
  }
  void tick(uint32_t now) {
    if(armed) {
      // Browser connectivity does not change torque or cancel an accepted move.
      for(int i=0;i<N;++i) if(state[i].seen && uint32_t(now-state[i].stamp)>FRESH_MS) { release("feedback_lost"); return; }
      if(gripping && uint32_t(now-gripStarted)>GRIP_TIMEOUT_MS) { release("grip_timeout"); return; }
    }
    if(uint32_t(now-lastPoll)<8) return;
    lastPoll=now; const int i=cursor; cursor=(cursor+1)%N;
    auto& s=state[i]; const auto& c=config[i]; Sample sample;
    const bool received=bus.read(c.id,sample);
    s.readError=sample.readError;
    s.alarm=sample.alarm;
    if(!received || sample.position<0 || sample.position>1023 || abs(sample.load)>1023) {
      s.contact.reset(now,s.fb.position,s.contact.direction,s.contact.commanded);
      if(!s.readError) s.readError=1;
      if(s.failures<255) ++s.failures;
      if(s.failures>=3) { s.online=false; if(armed) release("feedback_lost"); }
      return;
    }
    s.fb=sample; s.stamp=now; s.seen=true; s.online=true; s.failures=0;
    if(!armed) return;
    if(s.alarm) { release("servo_alarm"); return; }
    if(s.fb.mode!=0) { release("wrong_mode"); return; }
    if(s.fb.temperature>=65) { release("overtemperature"); return; }
    if(s.fb.position<std::min(c.open,c.close)-12 || s.fb.position>std::max(c.open,c.close)+12) { release("position_outside_limits"); return; }
    const int load=abs(s.fb.load);
    if(load>=std::min(1000,int(c.loadLimit)+200)) { release("overload"); return; }
    if(s.phase==Phase::Moving || s.phase==Phase::Gripping) {
      const bool isGrip=s.phase==Phase::Gripping;
      const int goal=raw(c,s.target), delta=goal-s.fb.position;
      const int dir=(delta>0)-(delta<0);
      if(abs(delta)>3 && dir!=s.contact.direction)
        s.contact.reset(now,s.fb.position,dir,s.contact.commanded);
      s.contact.observe(now,s.fb.load,s.fb.position,c.loadLimit);
      if(abs(delta)<=3) { s.phase=isGrip?Phase::Limit:Phase::Held; }
      else if(s.contact.confirmed(now,c.loadLimit,c.speed)) {
        // Only confirmed high load AND lack of progress latches position hold.
        // Do not unload before confirmation, or PWM feedback can erase evidence.
        if(!bus.move(c.id,s.fb.position,c.speed)) { release("write_failed"); return; }
        s.contact.commanded=s.fb.position;
        s.hits=1; s.phase=Phase::Contact; s.target=closure(c,s.fb.position);
      } else {
        s.hits=0;
        if(uint32_t(now-s.lastStep)>=STEP_MS) {
          // Round-robin feedback does not revisit a finger exactly every 40 ms.
          // Use elapsed time, but never catch up a long pause with a large jump.
          const uint32_t elapsed=std::min(uint32_t(now-s.lastStep),MAX_STEP_INTERVAL_MS);
          const int step=std::min(isGrip?MAX_GRIP_STEP:MAX_MOVE_STEP,
                                  std::max(1,int(c.speed)*int(elapsed)/1000));
          const int next=s.fb.position+(delta>0?1:-1)*std::min(abs(delta),step);
          if(!bus.move(c.id,next,c.speed)) { release("write_failed"); return; }
          s.contact.commanded=next;
          s.lastStep=now;
        }
      }
    }
    if(gripping) {
      bool moving=false;
      for(int k=0;k<N;++k) if(state[k].phase==Phase::Gripping) moving=true;
      gripping=moving;
    }
  }
  void resetSamples() { for(int i=0;i<N;++i) state[i]=State(); fault=""; }
 private:
  Bus& bus; uint32_t lastPoll=0; int cursor=0;
};
} // namespace hand
