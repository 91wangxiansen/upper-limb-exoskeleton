#include "../ServoDriver/HandCore.h"
#include <assert.h>
struct FakeBus : hand::Bus {
  hand::Sample feedback[4]; bool enabled[4]{}; int commands[4]{}; int bad=-1;
  int maxStep=0,lastSpeed[4]{};
  bool blocked[4]{};
  FakeBus() { for(int i=0;i<4;i++){auto c=hand::defaults(i);feedback[i].position=c.open;feedback[i].mode=0;feedback[i].temperature=28;} }
  bool read(uint8_t id,hand::Sample& s) override { int i=id-1;if(i==bad)return false;s=feedback[i];return true; }
  bool move(uint8_t id,int p,int speed) override {commands[id-1]++;maxStep=std::max(maxStep,abs(p-feedback[id-1].position));lastSpeed[id-1]=speed;if(!blocked[id-1])feedback[id-1].position=p;return true;}
  bool torque(uint8_t id,bool on) override {enabled[id-1]=on;return true;}
};
static_assert(sizeof(hand::Config)==10,"Keep saved NVS calibration layout compatible");
int travel(int speed,bool grip) {
  FakeBus b;hand::Controller c(b);uint32_t now=80;
  for(int i=0;i<4;i++){c.config[i].verified=1;c.config[i].speed=speed;}
  for(uint32_t t=8;t<now;t+=8)c.tick(t);
  if(!c.arm(now))return -1;
  const int start=b.feedback[0].position;
  float q[4]={1,1,1,1};
  if(grip){if(!c.grip(now))return -2;}else if(!c.targets(q,now))return -3;
  if(!c.moving())return -4;
  for(int k=0;k<48;k++){now+=8;c.beat(now);c.tick(now);}
  if(!c.armed||b.maxStep>(grip?hand::MAX_GRIP_STEP:hand::MAX_MOVE_STEP)||b.lastSpeed[0]!=speed)return -5;
  return b.feedback[0].position-start;
}
extern "C" int runTests() {
  FakeBus b;hand::Controller c(b);uint32_t now=0;
  for(int i=0;i<4;i++)c.config[i].verified=1;
  for(now=8;now<80;now+=8)c.tick(now);
  if(!c.arm(now))return 1;
  if(!c.grip(now))return 2;
  for(int k=0;k<20;k++){now+=8;c.beat(now);c.tick(now);}
  if(b.feedback[0].position<=250 || b.feedback[2].position>=800)return 3;
  b.feedback[0].load=-300; // Signed feedback must trigger the same contact threshold.
  b.blocked[0]=true;
  for(int k=0;k<48;k++){now+=8;c.beat(now);c.tick(now);}
  if(c.state[0].phase!=hand::Phase::Contact)return 4;
  if(c.state[1].phase!=hand::Phase::Gripping)return 5;
  int held=b.feedback[0].position;
  for(int k=0;k<12;k++){now+=8;c.beat(now);c.tick(now);}
  if(b.feedback[0].position!=held)return 6;
  c.hold(now);if(c.gripping)return 7;
  int p=b.feedback[1].position;
  for(int k=0;k<12;k++){now+=8;c.beat(now);c.tick(now);}
  if(b.feedback[1].position!=p)return 8;
  b.bad=2;
  for(int k=0;k<24;k++){now+=8;c.beat(now);c.tick(now);}
  if(c.armed)return 9;
  for(bool en:b.enabled)if(en)return 10;
  FakeBus b2;hand::Controller c2(b2);
  for(int i=0;i<4;i++)c2.config[i].verified=1;
  for(now=8;now<80;now+=8)c2.tick(now);
  if(!c2.arm(now))return 11;
  // Ten seconds of fresh servo feedback with no browser heartbeat keeps torque.
  const uint32_t noBeatStarted=now;
  for(;now<noBeatStarted+10000;now+=8)c2.tick(now);
  if(!c2.armed)return 12;
  for(bool en:b2.enabled)if(!en)return 25;
  float badPose[4]={0,0,NAN,0};
  if(c2.targets(badPose,now))return 13;
  // Servo feedback loss still disables torque independently of browser state.
  c2.tick(now+hand::FRESH_MS+1);
  if(c2.armed)return 26;
  FakeBus b3;hand::Controller c3(b3);
  for(int i=0;i<4;i++)c3.config[i].verified=1;
  b3.feedback[2].alarm=1;
  for(now=8;now<80;now+=8)c3.tick(now);
  if(!c3.state[2].online || c3.state[2].alarm!=1)return 14;
  if(c3.ready(now) || c3.arm(now))return 15;
  b3.feedback[2].alarm=0;
  b3.feedback[2].mode=-1;b3.feedback[2].readError=2;
  for(int k=0;k<8;k++){now+=8;c3.tick(now);}
  if(!c3.state[2].online || c3.state[2].readError!=2 || c3.ready(now))return 16;
  const int slow=travel(20,false),fast=travel(500,false);
  if(slow<=0||fast<slow*4)return 17;
  const int slowGrip=travel(20,true),fastGrip=travel(500,true);
  if(slowGrip<=0||fastGrip<slowGrip*4)return 18;
  FakeBus b4;hand::Controller c4(b4);
  for(int i=0;i<4;i++){c4.config[i].verified=1;c4.config[i].speed=500;}
  for(now=8;now<80;now+=8)c4.tick(now);
  if(!c4.arm(now))return 19;
  float far[4]={1,1,1,1};c4.targets(far,now);
  now+=200;c4.beat(now);c4.tick(now);
  if(!c4.armed||b4.maxStep>hand::MAX_MOVE_STEP)return 20;
  c4.hold(now);if(c4.moving())return 21;
  for(int i=0;i<4;i++)c4.config[i].speed=20;
  if(!hand::validConfig(c4.config))return 22;
  c4.config[0].speed=19;if(hand::validConfig(c4.config))return 23;
  c4.config[0].speed=501;if(hand::validConfig(c4.config))return 24;
  FakeBus b5;hand::Controller c5(b5);
  for(int i=0;i<4;i++)c5.config[i].verified=1;
  for(now=8;now<80;now+=8)c5.tick(now);
  if(!c5.arm(now))return 27;
  b5.feedback[0].load=-500;
  for(int k=0;k<4;k++){now+=8;c5.tick(now);}
  if(c5.armed)return 28; // Negative severe overload still disables torque.
  FakeBus b6;hand::Controller c6(b6);
  for(int i=0;i<4;i++)c6.config[i].verified=1;
  for(now=8;now<80;now+=8)c6.tick(now);
  if(!c6.arm(now))return 29;
  b6.feedback[1].temperature=65;
  for(int k=0;k<4;k++){now+=8;c6.tick(now);}
  if(c6.armed)return 30;
  return 0;
}
#ifdef NATIVE_MAIN
int main(){return runTests();}
#endif

