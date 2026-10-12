// 小车前后移动测试：等 8 秒（留时间拨开关、让开手），前进 2cm，停，后退 2cm，停。只跑一次。

#include <Arduino.h>
#include <Wire.h>

#include "Mecanum.h"
#include "BoardDriver.h"

constexpr float kAxleWidthM = 0.140f;     // 与官方运动例程一致
constexpr float kAxleLengthM = 0.145f;
constexpr float kWheelDiameterM = 0.065f;

constexpr float kMoveDistanceM = 0.02f;   // 2cm
constexpr float kMoveSpeedMps = 0.05f;    // 慢速
constexpr uint32_t kStartDelayMs = 8000;
constexpr uint32_t kMotionTimeoutMs = 4000;
constexpr uint16_t kPauseMs = 800;
constexpr uint16_t kMinBatteryMv = 6800;  // 说明书：低于 6.8V 要充电

static bool moveOnce(float speedMps, const char *label) {
  Serial.print(label);
  bool accepted = mecanumMoveDistance(speedMps, 0.0f, 0.0f, kMoveDistanceM);
  bool done = accepted && mecanumWaitDone(kMotionTimeoutMs, 100);
  mecanumStop(true);
  Serial.println(done ? " 完成" : " 失败或超时");
  return done;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);

  mecanumInit(kAxleWidthM, kAxleLengthM, kWheelDiameterM);
  mecanumStop(false);

  Serial.println("8 秒后开始：前进 2cm，再后退 2cm");
  setBuzzer(1200, 50);
  delay(100);
  setBuzzer(0, 0);
  delay(kStartDelayMs);

  uint16_t batteryMv = readBatteryMv();
  Serial.print("电池电压 mV=");
  Serial.println(batteryMv);
  if (batteryMv < kMinBatteryMv) {
    Serial.println("电池电压过低或开关在 OFF（电机没有电池供电），不执行移动");
    return;
  }

  bool ok = moveOnce(kMoveSpeedMps, "前进 2cm");
  delay(kPauseMs);
  ok = moveOnce(-kMoveSpeedMps, "后退 2cm") && ok;

  Serial.println(ok ? "测试结束" : "测试出现问题");
}

void loop() {
  delay(1000);
}
