#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <algorithm>

namespace hand {
// Initial detection settings; these are not persisted servo speed/load settings.
constexpr uint32_t CONTACT_CONFIRM_MS = 120, CONTACT_STILL_MS = 160;
constexpr int CONTACT_PROGRESS_STEPS = 3;
struct ContactDetector {
  int loads[3]{}, filteredLoad = 0, rawLoad = 0;
  bool votes[5]{};
  uint32_t voteTime[5]{}, progressAt = 0, sampleAt = 0, noProgressMs = 0;
  uint8_t loadCount = 0, loadNext = 0, voteCount = 0, voteNext = 0, highVotes = 0;
  int anchor = 0, direction = 0, commanded = -1, positionError = 0;

  void reset(uint32_t now,int position,int dir,int command=-1) {
    *this=ContactDetector();
    progressAt=now; anchor=position; direction=dir;
    commanded=command<0?position:command;
  }
  void observe(uint32_t now,int load,int position,int limit) {
    sampleAt=now; rawLoad=load;
    if(direction*(position-anchor)>=CONTACT_PROGRESS_STEPS) {
      anchor=position; progressAt=now;
    }
    noProgressMs=uint32_t(now-progressAt);
    positionError=direction*(commanded-position);
    loads[loadNext]=abs(load); loadNext=(loadNext+1)%3;
    if(loadCount<3) ++loadCount;
    if(loadCount<3) return;
    int a=loads[0],b=loads[1],c=loads[2];
    if(a>b)std::swap(a,b);
    if(b>c)std::swap(b,c);
    if(a>b)std::swap(a,b);
    filteredLoad=b;
    if(voteCount==5) highVotes-=int(votes[voteNext]);
    else ++voteCount;
    votes[voteNext]=filteredLoad>=limit;
    highVotes+=int(votes[voteNext]); voteTime[voteNext]=now;
    voteNext=(voteNext+1)%5;
  }
  bool confirmed(uint32_t now,int limit,int speed) const {
    // At low speeds allow enough time for three encoder steps of progress.
    const uint32_t stillWindow=std::max(CONTACT_STILL_MS,uint32_t(6000/std::max(20,speed)));
    return direction!=0 && loadCount==3 && voteCount==5 && highVotes>=3 &&
      filteredLoad>=limit && uint32_t(now-voteTime[voteNext])>=CONTACT_CONFIRM_MS &&
      noProgressMs>=stillWindow && positionError>0;
  }
};
} // namespace hand
