// 寻物小车网页控制：小车自己开热点，手机连上后打开 http://192.168.4.1
// 功能：摇杆、定距直线/定角转向、开始寻物（当前位置记为起点）、停止、返回起点。
// 网页只投递指令，所有硬件动作在 Robot 的独立线程里执行，所以动作中也能随时停止。

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#include "ArmControl.h"
#include "BoardDriver.h"
#include "Robot.h"
#include "Vision.h"
#include "WebPage.h"

static constexpr char kApSsid[] = "miniAuto-JOY";
static constexpr char kApPassword[] = "miniauto88";
static constexpr float kMaxStepCm = 50.0f;
static constexpr float kMaxTurnDeg = 180.0f;

static WebServer s_server(80);

static void reply(const char *text) {
  s_server.send(200, "text/plain; charset=utf-8", text);
}

// 严格解析数字：整串都必须是有限数字（空串、"abc"、"10abc"、"nan" 都不合法）
static bool parseNumber(const String &text, float &out) {
  if (text.length() == 0) return false;
  char *end = nullptr;
  float v = strtof(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || !isfinite(v)) return false;
  out = v;
  return true;
}

// 爪子角度只接受 90（闭合）~ 张开上限，缺参数或不合法都拒绝，不会被当成 0 再限幅成"完全闭合"
static bool validClawAngle(bool hasV, float v) {
  return hasV && v >= ARM_CLAW_CLOSED_ANGLE && v <= ARM_CLAW_OPEN_ANGLE;
}

static void handleCmd() {
  String c = s_server.arg("c");
  String dir = s_server.arg("dir");
  float v = 0.0f;
  bool hasV = parseNumber(s_server.arg("v"), v);
  Command cmd = {CMD_NONE, dir.length() > 0 ? dir[0] : '\0', v};

  if (c == "fetch") {
    cmd.type = CMD_FETCH;
  } else if (c == "return") {
    cmd.type = CMD_RETURN;
  } else if (c == "step" && cmd.dir != '\0' && strchr("fblr", cmd.dir) && v > 0 && v <= kMaxStepCm) {
    cmd.type = CMD_STEP;
  } else if (c == "turn" && cmd.dir != '\0' && strchr("lr", cmd.dir) && v > 0 && v <= kMaxTurnDeg) {
    cmd.type = CMD_TURN;
  } else if (c == "release") {
    robotRelease();  // 安全功能：不排队、不检查电池，正在寻物也马上停车并卸力
    reply("已卸力");
    return;
  } else if (c == "claw" && validClawAngle(hasV, v)) {
    cmd.type = CMD_CLAW;
  } else if (c == "lift" && hasV && v >= ARM_HEIGHT_MIN_ANGLE && v <= ARM_HEIGHT_MAX_ANGLE) {
    cmd.type = CMD_LIFT;
  } else {
    reply("指令参数不正确");
    return;
  }
  reply(robotPost(cmd) ? "已收到" : "小车正忙，请先停止或等待完成");
}

static void handleJoy() {
  float x = 0.0f;
  float y = 0.0f;
  if (!parseNumber(s_server.arg("x"), x) || !parseNumber(s_server.arg("y"), y)) {
    s_server.send(400, "text/plain; charset=utf-8", "摇杆参数不正确");  // 不更新摇杆，400ms 后自动停车
    return;
  }
  robotJoystick(x, y, s_server.arg("m") == "1");
  s_server.send(204);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);
  setBuzzer(0, 0);  // 蜂鸣器暂时关闭；必须在 robotBegin 启动动作线程之前，免得两个线程同时用 I2C

  robotBegin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  Serial.print("热点 ");
  Serial.print(kApSsid);
  Serial.print(" 密码 ");
  Serial.print(kApPassword);
  Serial.print("，打开 http://");
  Serial.println(WiFi.softAPIP());

  s_server.on("/", [] { s_server.send_P(200, "text/html; charset=utf-8", kWebPage); });
  s_server.on("/cmd", handleCmd);
  s_server.on("/stop", [] { robotStop(); reply("已发送停止"); });
  s_server.on("/joy", handleJoy);
  s_server.on("/status", [] { s_server.send(200, "application/json; charset=utf-8", robotStatusJson()); });
  s_server.begin();
}

// 串口指令（方便电脑通过 USB 远程测试）：fetch / grab（只夹不回）/ return / stop / claw <度> / lift <度> / check on|off / color 0|1|2 / release（舵机卸力） / status / f|b|l|r <厘米> / tl|tr <度>
// 不需要排队、马上生效的串口指令；处理了返回 true
static bool handleImmediateSerial(const String &name, const String &line, bool hasV, float v) {
  if (name == "stop") {
    robotStop();
    Serial.println("已发送停止");
  } else if (name == "release") {
    robotRelease();
    Serial.println("已发送卸力");
  } else if (name == "check") {
    bool on = line.endsWith("on");
    robotSetGrabCheck(on);
    Serial.println(on ? "夹取自检：开" : "夹取自检：关");
  } else if (name == "color" && hasV && v >= 0 && v <= 2) {
    visionSetColor((uint8_t)v);
    Serial.printf("目标颜色改为 %d（0 红 1 蓝 2 绿），下次读取时生效\n", (int)v);
  } else if (name == "look") {
    robotLook();
  } else if (name == "status") {
    Serial.println(robotStatusJson());
  } else {
    return false;
  }
  return true;
}

static void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  int space = line.indexOf(' ');
  String name = space < 0 ? line : line.substring(0, space);
  float v = 0.0f;
  bool hasV = space >= 0 && parseNumber(line.substring(space + 1), v);

  if (handleImmediateSerial(name, line, hasV, v)) return;

  Command cmd = {CMD_NONE, '\0', v};
  if (name == "fetch") {
    cmd.type = CMD_FETCH;
  } else if (name == "grab") {
    cmd.type = CMD_GRAB;
  } else if (name == "claw" && validClawAngle(hasV, v)) {
    cmd.type = CMD_CLAW;
  } else if (name == "lift" && hasV && v >= ARM_HEIGHT_MIN_ANGLE && v <= ARM_HEIGHT_MAX_ANGLE) {
    cmd.type = CMD_LIFT;
  } else if (name == "return") {
    cmd.type = CMD_RETURN;
  } else if (name.length() == 1 && strchr("fblr", name[0]) && hasV && v > 0 && v <= kMaxStepCm) {
    cmd = {CMD_STEP, name[0], v};
  } else if ((name == "tl" || name == "tr") && hasV && v > 0 && v <= kMaxTurnDeg) {
    cmd = {CMD_TURN, name[1], v};
  } else {
    Serial.println("未知指令");
    return;
  }
  Serial.println(robotPost(cmd) ? "已收到" : "小车正忙");
}

void loop() {
  s_server.handleClient();
  static String s_line;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      handleSerialLine(s_line);
      s_line = "";
    } else if (s_line.length() < 64) {
      s_line += ch;
    }
  }
  delay(2);
}
