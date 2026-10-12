// 小车网页摇杆：小车自己开热点，手机/电脑连上后打开 http://192.168.4.1 用摇杆开车。
// 安全：速度限幅；超过 kCommandTimeoutMs 没收到指令就自动刹车。

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#include "Mecanum.h"
#include "BoardDriver.h"

constexpr float kAxleWidthM = 0.140f;   // 与官方运动例程一致
constexpr float kAxleLengthM = 0.145f;
constexpr float kWheelDiameterM = 0.065f;

constexpr char kApSsid[] = "miniAuto-JOY";
constexpr char kApPassword[] = "miniauto88";

constexpr float kMaxSpeedMps = 0.12f;       // 摇杆推到底的最大线速度
constexpr float kMaxTurnRadS = 1.0f;        // 摇杆推到底的最大转速
constexpr uint32_t kCommandTimeoutMs = 400;
constexpr uint16_t kMinBatteryMv = 6800;

static const char kPage[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>小车摇杆</title>
<style>
  :root{color-scheme:light dark;--bg:#f4f4f2;--fg:#1c1c1a;--pad:#dcdcd6;--knob:#2b6cb0}
  @media(prefers-color-scheme:dark){:root{--bg:#161614;--fg:#eee;--pad:#2c2c28;--knob:#63a4ff}}
  html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font-family:-apple-system,sans-serif;touch-action:none}
  main{display:flex;flex-direction:column;align-items:center;gap:16px;padding:16px}
  #pad{width:min(80vw,320px);aspect-ratio:1;border-radius:50%;background:var(--pad);position:relative}
  #knob{width:30%;aspect-ratio:1;border-radius:50%;background:var(--knob);position:absolute;left:35%;top:35%}
  button{font-size:18px;padding:12px 20px;border-radius:12px;border:1px solid var(--fg);background:transparent;color:var(--fg)}
  button.on{background:var(--knob);color:#fff;border-color:var(--knob)}
  #st{font-size:14px;opacity:.7;min-height:1.2em}
</style></head><body><main>
  <h2>小车摇杆</h2>
  <div id="pad"><div id="knob"></div></div>
  <div><button id="mode">模式：转向</button> <button id="stop">停止</button></div>
  <div id="st">松手即停</div>
</main>
<script>
const pad=document.getElementById('pad'),knob=document.getElementById('knob'),st=document.getElementById('st');
let x=0,y=0,strafe=false,active=false,timer=null;
function send(){fetch('/drive?x='+x.toFixed(2)+'&y='+y.toFixed(2)+'&m='+(strafe?1:0)).then(r=>r.text()).then(t=>{st.textContent=t}).catch(()=>{st.textContent='连接断开'})}
function setKnob(){const r=pad.clientWidth/2,k=knob.clientWidth/2;knob.style.left=(r+x*(r-k)-k)+'px';knob.style.top=(r-y*(r-k)-k)+'px'}
function move(e){const b=pad.getBoundingClientRect(),r=b.width/2,k=knob.clientWidth/2;
  let dx=(e.clientX-b.left-r)/(r-k),dy=-(e.clientY-b.top-r)/(r-k);const n=Math.hypot(dx,dy);if(n>1){dx/=n;dy/=n}
  x=dx;y=dy;setKnob()}
pad.addEventListener('pointerdown',e=>{active=true;pad.setPointerCapture(e.pointerId);move(e);send();timer=setInterval(send,100)});
pad.addEventListener('pointermove',e=>{if(active)move(e)});
function release(){if(!active)return;active=false;clearInterval(timer);x=0;y=0;setKnob();fetch('/stop').then(r=>r.text()).then(t=>{st.textContent=t})}
pad.addEventListener('pointerup',release);pad.addEventListener('pointercancel',release);
document.getElementById('stop').onclick=release;
const mb=document.getElementById('mode');
mb.onclick=()=>{strafe=!strafe;mb.textContent='模式：'+(strafe?'平移':'转向');mb.classList.toggle('on',strafe)};
setKnob();
</script></body></html>)HTML";

static WebServer s_server(80);
static uint32_t s_lastCommandMs = 0;
static bool s_moving = false;
static bool s_batteryOk = false;

static float clampUnit(float v) {
  return constrain(v, -1.0f, 1.0f);
}

static void handleRoot() {
  s_server.send_P(200, "text/html; charset=utf-8", kPage);
}

static void handleDrive() {
  if (!s_batteryOk) {
    s_server.send(200, "text/plain; charset=utf-8", "电池电压过低或开关在 OFF，已禁止移动");
    return;
  }
  float x = clampUnit(s_server.arg("x").toFloat());
  float y = clampUnit(s_server.arg("y").toFloat());
  bool strafe = s_server.arg("m") == "1";

  float vx = y * kMaxSpeedMps;
  float vy = strafe ? -x * kMaxSpeedMps : 0.0f;   // 摇杆向右 = 向右平移（vy 为负）
  float wz = strafe ? 0.0f : -x * kMaxTurnRadS;   // 摇杆向右 = 右转（wz 为负）

  mecanumSetVelocity(vx, vy, wz);
  s_moving = (x != 0.0f || y != 0.0f);
  s_lastCommandMs = millis();
  s_server.send(200, "text/plain; charset=utf-8", "行驶中");
}

static void handleStop() {
  mecanumStop(true);
  s_moving = false;
  s_server.send(200, "text/plain; charset=utf-8", "已停止");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);

  mecanumInit(kAxleWidthM, kAxleLengthM, kWheelDiameterM);
  mecanumStop(false);

  uint16_t batteryMv = readBatteryMv();
  s_batteryOk = batteryMv >= kMinBatteryMv;
  Serial.print("电池电压 mV=");
  Serial.println(batteryMv);
  if (!s_batteryOk) {
    Serial.println("电池电压过低或开关在 OFF，网页将禁止移动");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  Serial.print("热点 ");
  Serial.print(kApSsid);
  Serial.print(" 密码 ");
  Serial.print(kApPassword);
  Serial.print("，打开 http://");
  Serial.println(WiFi.softAPIP());

  s_server.on("/", handleRoot);
  s_server.on("/drive", handleDrive);
  s_server.on("/stop", handleStop);
  s_server.begin();

  setBuzzer(1200, 50);
  delay(100);
  setBuzzer(0, 0);
}

void loop() {
  s_server.handleClient();
  if (s_moving && millis() - s_lastCommandMs > kCommandTimeoutMs) {
    mecanumStop(true);
    s_moving = false;
    Serial.println("指令超时，已刹车");
  }
}
