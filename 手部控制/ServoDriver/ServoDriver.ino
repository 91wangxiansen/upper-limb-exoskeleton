#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "src/SCServo/SCServo.h"
#include <esp_random.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "HandCore.h"
#include "WebAssets.h"
#include "HandOLED.h"

SCSCL sc;
class SC09Bus : public hand::Bus {
public:
  bool read(uint8_t id, hand::Sample& s) override {
    // Read only position through temperature (56..63). Some SC09 revisions
    // cannot return the extended block through current registers 69..70.
    uint8_t feedback[8];
    sc.Error=0;
    const int count=sc.Read(id,SCSCL_PRESENT_POSITION_L,feedback,sizeof(feedback));
    s.alarm=sc.Error;
    if(count!=sizeof(feedback)) { s.readError=1; return false; }
    s.position=(int(feedback[0])<<8)|feedback[1];
    int speed=(int(feedback[2])<<8)|feedback[3];
    s.speed=(speed&0x8000)?-(speed&0x7fff):speed;
    int load=(int(feedback[4])<<8)|feedback[5];
    s.load=(load&0x400)?-(load&0x3ff):load;
    s.voltage=feedback[6]; s.temperature=feedback[7];
    uint8_t limits[4];
    sc.Error=0;
    if(sc.Read(id,SCSCL_MIN_ANGLE_LIMIT_L,limits,4)!=4) {
      s.alarm|=sc.Error; s.readError=2; return true;
    }
    s.alarm|=sc.Error;
    s.mode=(limits[0]|limits[1]|limits[2]|limits[3])?0:3;
    return true;
  }
  bool move(uint8_t id,int p,int speed) override { return sc.WritePos(id,p,0,speed)>0 && !sc.Error; }
  bool torque(uint8_t id,bool on) override { return sc.EnableTorque(id,on)>0 && !sc.Error; }
};
SC09Bus bus;
hand::Controller handCtl(bus);
WebServer http(80);
Preferences nv;
bool storage=false;
struct Gesture { char name[49]; uint8_t q[4]; };
Gesture poses[16]{};
String owner,nonce;
uint32_t lastOwner=0, motionEpoch=1;
SemaphoreHandle_t controlMutex;
String responseBody;
int responseCode=200;
HandOLED oled;
uint8_t detectedIDs[20]{};
int detectedCount=0;
bool scanDone=false;
String idMessage;
char oledNote[22]{};
uint32_t oledNoteAt=0;

void idNotice(const String& message,const char* shortMessage) {
  idMessage=message;
  snprintf(oledNote,sizeof(oledNote),"%s",shortMessage);
  oledNoteAt=millis();
}
void scanServoIDs() {
  detectedCount=0;
  for(int id=1;id<=20;++id) {
    if(sc.Ping(id)==id) detectedIDs[detectedCount++]=uint8_t(id);
  }
  scanDone=true;
}
void updateOLED() {
  static uint32_t lastUpdate=0;
  uint32_t now=millis();
  if(!oled.available() || !controlMutex || uint32_t(now-lastUpdate)<800) return;
  char rows[4][22]{};
  snprintf(rows[0],sizeof(rows[0]),"IP:%s",WiFi.softAPIP().toString().c_str());
  if(xSemaphoreTake(controlMutex,pdMS_TO_TICKS(2))!=pdTRUE) return;
  int onlineCount=0,minVoltage=10000;
  char symbols[4];
  for(int i=0;i<4;++i) {
    const auto& s=handCtl.state[i]; bool online=handCtl.fresh(i,now);
    symbols[i]=s.alarm?'!':online?(s.readError?'?':'+'):'-';
    if(online) { ++onlineCount; minVoltage=std::min(minVoltage,s.fb.voltage); }
  }
  snprintf(rows[1],sizeof(rows[1]),"ID %u%c %u%c %u%c %u%c",
    unsigned(handCtl.config[0].id),symbols[0],unsigned(handCtl.config[1].id),symbols[1],
    unsigned(handCtl.config[2].id),symbols[2],unsigned(handCtl.config[3].id),symbols[3]);
  if(onlineCount) snprintf(rows[2],sizeof(rows[2]),"ON:%d/4 V:%d.%d",onlineCount,minVoltage/10,minVoltage%10);
  else snprintf(rows[2],sizeof(rows[2]),"ON:0/4 V:--");
  if(oledNote[0] && uint32_t(now-oledNoteAt)<6000) snprintf(rows[3],sizeof(rows[3]),"%s",oledNote);
  else if(handCtl.fault[0]) snprintf(rows[3],sizeof(rows[3]),"ERR:%.17s",
    strcmp(handCtl.fault,"controller_lost")==0?"CTRL LINK LOST":handCtl.fault);
  else snprintf(rows[3],sizeof(rows[3]),"%s AP:%u",handCtl.gripping?"GRIP":handCtl.armed?"ARMED":"IDLE",unsigned(WiFi.softAPgetStationNum()));
  xSemaphoreGive(controlMutex);
  // I2C transfers happen after releasing the control mutex.
  oled.show(rows[0],rows[1],rows[2],rows[3]);
  lastUpdate=now;
}
String quote(String s) {
  s.replace("\\","\\\\"); s.replace("\"","\\\""); return "\""+s+"\"";
}
void answer(int code,String s) { responseCode=code; responseBody=s; }
void errorMsg(int code,const char* msg) { answer(code,"{\"error\":"+quote(msg)+"}"); }
bool changeServoID(int oldID,int newID) {
  scanServoIDs();
  if(detectedCount!=1 || detectedIDs[0]!=oldID) {
    idNotice("请只连接待修改的一只舵机，并核对原 ID", "ID:CHECK SINGLE SERVO"); return false;
  }
  if(sc.Ping(newID)==newID) {
    idNotice("目标 ID 已有设备响应", "ID:TARGET EXISTS"); return false;
  }
  sc.unLockEprom(oldID);
  if(sc.readByte(oldID,SCSCL_LOCK)!=0) {
    sc.LockEprom(oldID);
    idNotice("无法确认 EEPROM 解锁，未写入新 ID", "ID:UNLOCK FAILED"); return false;
  }
  // Some firmware acknowledges the write under the new ID, so verify by readback.
  sc.writeByte(oldID,SCSCL_ID,newID);
  bool renamed=sc.Ping(newID)==newID;
  sc.LockEprom(renamed?newID:oldID);
  if(!renamed) {
    // Also try locking the destination in case only its acknowledgement was lost.
    sc.LockEprom(newID);
    scanServoIDs(); handCtl.resetSamples();
    idNotice("改号未确认，请重新搜索实际 ID 后再操作", "ID:VERIFY FAILED"); return false;
  }
  bool locked=sc.readByte(newID,SCSCL_LOCK)==1;
  bool idMatches=sc.readByte(newID,SCSCL_ID)==newID;
  bool oldStillPresent=sc.Ping(oldID)==oldID;
  scanServoIDs(); handCtl.resetSamples();
  if(!locked || !idMatches || oldStillPresent) {
    idNotice(String("新 ID ")+newID+" 已响应，但锁定或唯一性未确认，请检查搜索结果", "ID:CHECK RESULT"); return false;
  }
  char note[22]; snprintf(note,sizeof(note),"ID %d>%d OK",oldID,newID);
  idNotice(String("修改成功：ID ")+oldID+" -> "+newID+"，已回读验证并锁定 EEPROM",note);
  return true;
}
bool number(String key,int low,int high,int& v) {
  String s=http.arg(key); char* end; long n=strtol(s.c_str(),&end,10);
  if(!s.length() || end==s.c_str() || *end || n<low || n>high) return false;
  v=int(n); return true;
}
void sendState() {
  String s; s.reserve(4300); uint32_t now=millis();
  s="{\"epoch\":"+String(motionEpoch)+",\"token\":"+quote(nonce)+",\"owner\":"+quote(owner)+",\"armed\":"+String(int(handCtl.armed))+
    ",\"ready\":"+String(int(handCtl.ready(now)))+",\"gripping\":"+String(int(handCtl.gripping))+
    ",\"fault\":"+quote(handCtl.fault)+",\"speedControl\":1,\"contactFilter\":1,\"disconnectRelease\":0,\"leaseMs\":0"+
    ",\"heartbeatAgeMs\":"+String(handCtl.armed?long(uint32_t(now-lastOwner)):-1L)+",\"oled\":"+String(int(oled.available()))+
    ",\"scanDone\":"+String(int(scanDone))+",\"idMessage\":"+quote(idMessage)+",\"detectedIDs\":[";
  for(int i=0;i<detectedCount;++i) { if(i) s+=","; s+=String(detectedIDs[i]); }
  s+="],\"fingers\":[";
  for(int i=0;i<4;i++) {
    auto &c=handCtl.config[i]; auto &f=handCtl.state[i];
    if(i) s+=",";
    s+="{\"id\":"+String(c.id)+",\"open\":"+String(c.open)+",\"close\":"+String(c.close)+",\"limit\":"+String(c.loadLimit)+
       ",\"speedSet\":"+String(c.speed)+",\"verified\":"+String(c.verified)+",\"online\":"+String(int(handCtl.fresh(i,now)))+",\"p\":"+String(f.fb.position)+
       ",\"load\":"+String(f.fb.load)+",\"speed\":"+String(f.fb.speed)+",\"temp\":"+String(f.fb.temperature)+
       ",\"v\":"+String(f.fb.voltage/10.0,1)+",\"phase\":"+quote(hand::phaseName(f.phase))+
       ",\"alarm\":"+String(f.alarm)+",\"readError\":"+String(f.readError)+
       ",\"failures\":"+String(f.failures)+",\"mode\":"+String(f.fb.mode)+
       ",\"q\":"+String(roundf(100*hand::closure(c,f.fb.position)),0)+
       ",\"contactRawLoad\":"+String(f.contact.rawLoad)+",\"contactFilteredLoad\":"+String(f.contact.filteredLoad)+
       ",\"contactVotes\":"+String(f.contact.highVotes)+",\"contactNoProgressMs\":"+String(f.contact.noProgressMs)+
       ",\"contactPositionError\":"+String(f.contact.positionError)+",\"commandedP\":"+String(f.contact.commanded)+
       ",\"contactSampleMs\":"+String(f.contact.sampleAt)+"}";
  }
  s+="],\"gestures\":["; bool first=true;
  for(int i=0;i<16;i++) if(poses[i].name[0]) {
    if(!first) s+=","; first=false;
    s+="{\"slot\":"+String(i)+",\"name\":"+quote(poses[i].name)+",\"q\":[";
    for(int j=0;j<4;j++){ if(j) s+=","; s+=String(poses[i].q[j]); } s+="]}";
  }
  answer(200,s+"]}");
}
void command() {
  String op=http.arg("op"), client=http.header("X-Client");
  if(http.header("X-Token")!=nonce || client.length()<8 || client.length()>64) { errorMsg(403,"请刷新页面"); return; }
  if(op=="release") { ++motionEpoch; handCtl.release(); owner=""; }
  else if(op=="stop") { ++motionEpoch; handCtl.hold(millis()); }
  else {
    if(owner.length() && owner!=client) { errorMsg(409,"另一终端持有控制权"); return; }
    if(op=="beat" && owner!=client) { errorMsg(409,"控制权已释放"); return; }
    if(op=="pose" || op=="arm" || op=="grip") {
      int epoch;
      if(!number("epoch",1,2147483647,epoch) || uint32_t(epoch)!=motionEpoch){errorMsg(409,"姿态已过期，请重新操作");return;}
    }
    owner=client; lastOwner=millis(); handCtl.beat(lastOwner);
    if(op=="scan" || op=="set_id") {
      if(handCtl.armed || handCtl.gripping) { errorMsg(409,"请先释放，再搜索或修改舵机 ID"); return; }
      if(op=="scan") {
        scanServoIDs();
        char note[22]; snprintf(note,sizeof(note),"SCAN:%d ID FOUND",detectedCount);
        idNotice(String("搜索完成，发现 ")+detectedCount+" 个响应 ID",note);
      } else {
        int oldID,newID;
        if(!number("old_id",1,20,oldID) || !number("new_id",1,20,newID) || oldID==newID) {
          errorMsg(400,"原 ID 和新 ID 必须在 1～20 之间且不同"); return;
        }
        if(!changeServoID(oldID,newID)) { errorMsg(409,idMessage.c_str()); return; }
      }
    } else if(op=="arm") {
      if(!handCtl.arm(millis())) { errorMsg(409,"检查四指 ID、校准行程、当前负载及在线状态"); return; }
      ++motionEpoch;
    } else if(op=="grip") {
      if(!handCtl.grip(millis())) { errorMsg(409,"需要先启用且四指均在线"); return; }
      ++motionEpoch;
    } else if(op=="pose" || op=="save") {
      int values[4]; float q[4];
      for(int i=0;i<4;i++) {
        if(!number(String("q")+i,0,100,values[i])) { errorMsg(400,"无效姿态"); return; }
        q[i]=values[i]/100.f;
      }
      if(op=="pose") {
        if(!handCtl.targets(q,millis())) { errorMsg(409,"请先启用，抓握中请先停止"); return; }
      } else {
        String name=http.arg("name"); name.trim(); int slot=-1;
        for(int i=0;i<16;i++) if(name==poses[i].name){ slot=i; break; }
        if(slot<0) for(int i=0;i<16;i++) if(!poses[i].name[0]){ slot=i; break; }
        if(slot<0 || name.length()==0 || name.length()>48) { errorMsg(400,"手势已满或名称无效（最多 48 字节）"); return; }
        for(size_t i=0;i<name.length();i++) if(uint8_t(name[i])<32){ errorMsg(400,"名称包含控制字符"); return; }
        Gesture next{}; name.toCharArray(next.name,sizeof(next.name));
        for(int i=0;i<4;i++) next.q[i]=values[i];
        String key=String("g")+slot;
        if(!storage || nv.putBytes(key.c_str(),&next,sizeof(next))!=sizeof(next)){errorMsg(507,"保存失败");return;}
        poses[slot]=next;
      }
    } else if(op=="delete") {
      int slot; if(!number("slot",0,15,slot) || !poses[slot].name[0]) { errorMsg(400,"无效手势"); return; }
      String key=String("g")+slot; if(!nv.remove(key.c_str())){errorMsg(507,"删除失败");return;} poses[slot]=Gesture{};
    } else if(op=="speed") {
      if(handCtl.gripping || handCtl.moving()){errorMsg(409,"请先停止并保持，再修改速度");return;}
      int speed;
      if(!number("speed",20,500,speed)){errorMsg(400,"速度范围为 20～500 编码步/秒");return;}
      hand::Config cfg[4]; memcpy(cfg,handCtl.config,sizeof(cfg));
      for(auto& c:cfg) c.speed=uint16_t(speed);
      if(!storage || nv.putBytes("config",cfg,sizeof(cfg))!=sizeof(cfg)){errorMsg(507,"保存速度失败");return;}
      memcpy(handCtl.config,cfg,sizeof(cfg));
    } else if(op=="config") {
      if(handCtl.armed){errorMsg(409,"请先释放再保存配置");return;}
      hand::Config cfg[4];
      for(int i=0;i<4;i++) {
        String k=String(i); int id,a,b,l;
        if(!number("id"+k,1,20,id)||!number("open"+k,0,1023,a)||!number("close"+k,0,1023,b)||!number("limit"+k,50,1000,l)){
          errorMsg(400,"参数范围有误");return;
        }
        int speed=handCtl.config[i].speed;
        if(http.hasArg("speed"+k) && !number("speed"+k,20,500,speed)){errorMsg(400,"速度范围为 20～500 编码步/秒");return;}
        cfg[i]=hand::defaults(i);cfg[i].id=id;cfg[i].open=a;cfg[i].close=b;cfg[i].loadLimit=l;cfg[i].speed=speed;cfg[i].verified=1;
      }
      if(!hand::validConfig(cfg)){errorMsg(400,"ID 必须互异，行程至少相差 20");return;}
      if(!storage || nv.putBytes("config",cfg,sizeof(cfg))!=sizeof(cfg)){errorMsg(507,"保存失败");return;}
      memcpy(handCtl.config,cfg,sizeof(cfg));handCtl.resetSamples();
    } else if(op!="beat"){errorMsg(400,"未知操作");return;}
  }
  answer(200,"{\"ok\":true}");
}
void serveControl(void (*handler)()) {
  xSemaphoreTake(controlMutex,portMAX_DELAY);
  handler();
  xSemaphoreGive(controlMutex);
  http.sendHeader("Cache-Control","no-store");
  http.send(responseCode,"application/json; charset=utf-8",responseBody);
}
void controlTask(void*) {
  for(;;) {
    xSemaphoreTake(controlMutex,portMAX_DELAY);
    // Keep ownership and torque while enabled, including a disconnected browser.
    // An inactive owner must not block a new page after torque is already off.
    if(!handCtl.armed && owner.length() && uint32_t(millis()-lastOwner)>3000) {
      owner=""; ++motionEpoch;
    }
    bool wasArmed=handCtl.armed;
    handCtl.tick(millis());
    if(wasArmed && !handCtl.armed) ++motionEpoch;
    xSemaphoreGive(controlMutex);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
void setup() {
  Serial.begin(115200);
  if(oled.begin()) oled.show("HAND EXO","STARTING...","OLED OK","");
  else Serial.println("OLED not found at 0x3C/0x3D");
  Serial1.begin(1000000,SERIAL_8N1,18,19); sc.pSerial=&Serial1; sc.IOTimeOut=20;
  storage=nv.begin("fourfinger2",false);
  hand::Config cfg[4];
  if(storage && nv.getBytesLength("config")==sizeof(cfg) && nv.getBytes("config",cfg,sizeof(cfg))==sizeof(cfg) && hand::validConfig(cfg)) memcpy(handCtl.config,cfg,sizeof(cfg));
  for(int i=0;i<16 && storage;i++) {
    String key=String("g")+i;
    if(nv.getBytesLength(key.c_str())==sizeof(Gesture)) nv.getBytes(key.c_str(),&poses[i],sizeof(Gesture));
    poses[i].name[48]=0;
    for(uint8_t q:poses[i].q) if(q>100) poses[i]=Gesture{};
  }
  nonce=String(esp_random(),HEX)+String(esp_random(),HEX);
  WiFi.mode(WIFI_AP); WiFi.softAP("Hand-Exo","handexo2026");
  const char* headers[]={"X-Token","X-Client"}; http.collectHeaders(headers,2);
  controlMutex=xSemaphoreCreateMutex();
  if(!controlMutex || xTaskCreate(controlTask,"handControl",6144,nullptr,3,nullptr)!=pdPASS) {
    Serial.println("Control task failed");
    oled.show("HAND EXO","CONTROL TASK FAILED","RESET BOARD",""); return;
  }
  http.on("/api/state",HTTP_GET,[]{serveControl(sendState);});
  http.on("/api/command",HTTP_POST,[]{serveControl(command);});
  http.onNotFound([]{
    String path=http.uri();if(path=="/") path="/index.html";
    for(const auto& a:WEB_ASSETS) if(path==a.path && http.method()==HTTP_GET) {
      http.sendHeader("Content-Encoding","gzip");
      http.send_P(200,a.mime,reinterpret_cast<const char*>(a.data),a.size);return;
    }
    http.send(404,"text/plain","Not found");
  });
  http.begin();Serial.println(WiFi.softAPIP());
  oled.show("HAND EXO",WiFi.softAPIP().toString().c_str(),"WEB READY","WAITING FEEDBACK");
}
void loop() { http.handleClient(); updateOLED(); delay(1); }
