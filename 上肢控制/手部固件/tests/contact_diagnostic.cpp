// Synthetic regression responses, not a calibrated physical SC09 model.
#include "../ServoDriver/HandCore.h"
#include <stdio.h>
#include <string.h>

struct DiagnosticBus : hand::Bus {
  hand::Sample feedback[4];
  bool enabled[4]{},blocked[4]{},spike[4]{},dropOnHold=true,failHold=false;
  int holds[4]{},commands[4]{},reads[4]{},failReads[4]{},spikeLoad=210;
  DiagnosticBus() {
    for(auto& f:feedback) {f.position=500;f.mode=0;f.temperature=25;}
  }
  bool read(uint8_t id,hand::Sample& s) override {
    const int i=id-1;++reads[i];
    if(failReads[i]>0){--failReads[i];return false;}
    s=feedback[i];
    if(spike[i]){s.load=spikeLoad;spike[i]=false;}
    return true;
  }
  bool move(uint8_t id,int p,int) override {
    const int i=id-1;++commands[i];
    if(enabled[i] && p==feedback[i].position) {
      if(failHold)return false;
      ++holds[i];if(dropOnHold)feedback[i].load=0;
    }
    if(!blocked[i])feedback[i].position=p;
    return true;
  }
  bool torque(uint8_t id,bool on) override {enabled[id-1]=on;return true;}
};
void run(hand::Controller& c,uint32_t& now,int ticks) {
  for(int i=0;i<ticks;i++){now+=8;c.tick(now);}
}
bool prepare(hand::Controller& c,uint32_t& now,int speed,int close,int limit=200) {
  for(auto& cfg:c.config){cfg.verified=1;cfg.open=500;cfg.close=close;cfg.speed=speed;cfg.loadLimit=limit;}
  run(c,now,10);return c.arm(now);
}
int scenario(int load,int speed,int close,bool drop,int limit) {
  DiagnosticBus b;hand::Controller c(b);uint32_t now=0;b.dropOnHold=drop;
  if(!prepare(c,now,speed,close,limit)||!c.grip(now))return 1;
  // Even a blocked finger must ignore one isolated nonsevere load spike.
  b.blocked[0]=true;b.spikeLoad=limit+10;b.spike[0]=true;
  run(c,now,64);
  if(c.state[0].phase!=hand::Phase::Gripping||b.holds[0])return 2;
  // Sustained load above threshold while position follows commands is not contact.
  b.blocked[0]=false;b.feedback[0].load=load;
  run(c,now,32);
  if(c.state[0].phase!=hand::Phase::Gripping||b.holds[0])return 3;
  // Sustained load with a pending command and no progress must latch.
  b.blocked[0]=true;const int latched=b.feedback[0].position;
  const uint32_t blockedAt=now;
  for(int k=0;k<64 && c.state[0].phase!=hand::Phase::Contact;k++)run(c,now,1);
  if(c.state[0].phase!=hand::Phase::Contact||b.holds[0]!=1)return 4;
  // Last measured forward progress can precede the synthetic block by one poll.
  if(now-blockedAt+32<hand::CONTACT_STILL_MS)return 5;
  const int commands=b.commands[0];b.feedback[0].load=0;
  run(c,now,32);
  if(b.feedback[0].position!=latched||b.commands[0]!=commands)return 6;
  if(c.state[1].phase!=hand::Phase::Gripping)return 7;
  if(c.config[0].loadLimit!=limit||c.config[0].speed!=speed)return 8;
  for(int i=1;i<4;i++){b.blocked[i]=true;b.feedback[i].load=limit;}
  run(c,now,64);
  if(c.gripping||!c.armed)return 9;
  for(const auto& s:c.state)if(s.phase!=hand::Phase::Contact)return 10;
  for(auto& f:b.feedback)f.load=0;
  run(c,now,2600);
  if(!c.armed||c.gripping||b.feedback[0].position!=latched)return 11;
  for(auto& block:b.blocked)block=false;
  if(!c.grip(now)||c.state[0].contact.loadCount!=0)return 12;
  run(c,now,8);
  if(b.feedback[0].position==latched)return 13;
  c.hold(now);float open[4]={0,0,0,0};
  if(!c.targets(open,now)||c.state[0].phase!=hand::Phase::Moving)return 14;
  return 0;
}
int liveTargets() {
  DiagnosticBus b;hand::Controller c(b);uint32_t now=0;
  if(!prepare(c,now,350,1000))return 20;
  float q[4]={1,1,1,1};
  if(!c.targets(q,now))return 21;
  b.blocked[0]=true;b.feedback[0].load=-210;
  for(int k=0;k<64 && c.state[0].phase!=hand::Phase::Contact;k++) {
    if(k%8==0 && !c.targets(q,now))return 22;
    run(c,now,1);
  }
  if(c.state[0].phase!=hand::Phase::Contact)return 23;
  b.feedback[0].load=0;b.blocked[0]=false;
  if(!c.targets(q,now)||c.state[0].contact.loadCount!=0)return 24;
  run(c,now,8);
  // Reversing a target discards the old direction's evidence.
  float reverse[4]={0,0,0,0};
  if(!c.targets(reverse,now)||c.state[0].contact.loadCount!=0||c.state[0].contact.direction!=-1)return 25;
  return 0;
}
int detectorTiming() {
  hand::ContactDetector d;d.reset(0,500,1);d.commanded=512;
  // Fast sample counts alone cannot bypass the time requirement.
  for(uint32_t t=1;t<=7;t++)d.observe(t,210,500,200);
  if(d.confirmed(7,200,350))return 30;
  // A single low sample must not erase otherwise sustained evidence.
  d.reset(0,500,1);d.commanded=512;
  for(int i=1;i<=7;i++)d.observe(i*32,i==5?0:210,500,200);
  if(!d.confirmed(224,200,350))return 31;
  // Sub-three-step position jitter does not count as forward progress.
  d.reset(0,500,1);d.commanded=512;
  for(int i=1;i<=7;i++)d.observe(i*32,210,500+i%2,200);
  if(!d.confirmed(224,200,350))return 32;
  // millis() rollover must preserve elapsed-time decisions.
  const uint32_t start=0xffffff80U;d.reset(start,500,1);d.commanded=512;
  for(int i=1;i<=7;i++)d.observe(start+uint32_t(i*32),210,500,200);
  if(!d.confirmed(start+224U,200,350))return 33;
  // Slow but real progress with a pending target must not look like a block.
  for(int speed:{20,350}) {
    d.reset(0,500,1);
    for(int i=1;i<=40;i++) {
      const int p=500+i/(speed==20?2:1);d.commanded=p+12;
      d.observe(i*32,210,p,200);
      if(d.confirmed(i*32,200,speed))return 34;
    }
  }
  return 0;
}
int overshootDirection() {
  DiagnosticBus b;hand::Controller c(b);uint32_t now=0;
  if(!prepare(c,now,350,1000))return 50;
  float q[4]={.5f,.5f,.5f,.5f};
  if(!c.targets(q,now))return 51;
  run(c,now,4);
  b.feedback[0].position=760;
  run(c,now,4);
  if(c.state[0].contact.direction!=-1||c.state[0].contact.loadCount!=1)return 52;
  return 0;
}
int failurePaths() {
  DiagnosticBus b;hand::Controller c(b);uint32_t now=0;
  if(!prepare(c,now,350,1000)||!c.grip(now))return 40;
  b.blocked[0]=true;b.feedback[0].load=210;
  run(c,now,16);b.failReads[0]=1;
  const int reads=b.reads[0];
  while(b.reads[0]==reads)run(c,now,1);
  if(c.state[0].contact.loadCount||c.state[0].contact.voteCount)return 41;
  b.failHold=true;run(c,now,64);
  if(c.armed||strcmp(c.fault,"write_failed"))return 42;
  DiagnosticBus low;hand::Controller lowCtl(low);now=0;
  if(!prepare(lowCtl,now,350,1000)||!lowCtl.grip(now))return 43;
  low.blocked[0]=true;low.feedback[0].load=199;
  run(lowCtl,now,128);
  if(lowCtl.state[0].phase!=hand::Phase::Gripping||low.holds[0])return 44;
  // Raw severe overload bypasses the low filtered value on its first read.
  low.feedback[0].load=-400;run(lowCtl,now,4);
  if(lowCtl.armed||strcmp(lowCtl.fault,"overload"))return 45;
  return 0;
}
int main() {
  int cases=0;
  for(int limit:{200,300})for(int sign:{-1,1})for(int extra:{0,10})for(int speed:{20,100,350,500})
    for(int close:{0,1000})for(bool drop:{true,false}) {
      const int load=sign*(limit+extra);
      const int result=scenario(load,speed,close,drop,limit);
      if(result){printf("FAIL: load=%d speed=%d close=%d drop=%d code=%d\n",load,speed,close,int(drop),result);return result;}
      ++cases;
    }
  for(auto check:{liveTargets,detectorTiming,failurePaths,overshootDirection}) {
    const int result=check();if(result){printf("FAIL: auxiliary code=%d\n",result);return result;}
  }
  printf("PASS: %d signed/direction/speed/drop cases; spikes, moving load, blocked contact, latch, live commands, timing, read/write failures.\n",cases);
  return 0;
}
