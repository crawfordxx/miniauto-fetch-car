// 升降臂测试：抬起 -> 放下 -> 抬起 -> 放下，每步停 1.5 秒。只动升降臂，不动爪子。
#include <Arduino.h>
#include <Wire.h>

#include "ArmControl.h"
#include "BoardDriver.h"

constexpr uint16_t kMoveTimeMs = 600;
constexpr uint32_t kStepWaitMs = 1500;
constexpr uint32_t kStartDelayMs = 5000;

// 等待期间持续调用 armControlUpdate，舵机才会按平滑曲线真正转动
static void pump(uint32_t waitMs) {
  uint32_t start = millis();
  while (millis() - start < waitMs) {
    armControlUpdate();
    delay(5);
  }
}

static void clawTo(int angle, const char *label) {
  Serial.print(label);
  Serial.print(" 目标角度=");
  Serial.print(angle);
  armControlSetClawAngle(angle, kMoveTimeMs);
  pump(kStepWaitMs);
  Serial.print(" 当前估计角度=");
  Serial.println(armControlGetEstimatedClawAngle());
}

static void heightTo(int angle, const char *label) {
  Serial.print(label);
  Serial.print(" 目标角度=");
  Serial.print(angle);
  armControlSetHeightAngle(angle, kMoveTimeMs);
  pump(kStepWaitMs);
  Serial.print(" 当前估计角度=");
  Serial.println(armControlGetEstimatedHeightAngle());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);

  Serial.println("5 秒后开始爪子开合测试");
  delay(kStartDelayMs);

  armControlInit();
  pump(kStepWaitMs);

  heightTo(ARM_HEIGHT_RAISED_ANGLE, "升降臂抬起");
  heightTo(ARM_HEIGHT_LOWER_ANGLE, "升降臂放下");
  heightTo(ARM_HEIGHT_RAISED_ANGLE, "升降臂抬起");
  heightTo(ARM_HEIGHT_LOWER_ANGLE, "升降臂放下");
  Serial.println("升降臂测试结束");
}

void loop() {
  armControlUpdate();
  delay(5);
}
