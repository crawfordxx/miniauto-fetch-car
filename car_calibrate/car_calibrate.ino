// 4 路编码电机自动校准（官方要求：不校准小车会走偏、转角不准）。
// 约 3 分钟，车轮会逐个正转反转，请把小车垫起来让轮子悬空。结果由底盘驱动板保存。
// 流程：等 10 秒 -> 检查电池 -> 校准 -> 打印每路电机的校准数据 -> 蜂鸣器提示。

#include <Arduino.h>
#include <Wire.h>

#include "BoardDriver.h"

constexpr uint32_t kStartDelayMs = 10000;
constexpr uint32_t kCalibrateTimeoutMs = 300000;
constexpr uint16_t kMinBatteryMv = 6800;
constexpr uint8_t kAllMotorsMask = 0x0F;

static void beep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    setBuzzer(1200, 50);
    delay(100);
    setBuzzer(0, 0);
    delay(100);
  }
}

static void printCalibration() {
  const char *names[4] = {"M1 左前", "M2 左后", "M3 右后", "M4 右前"};
  for (uint8_t ch = 0; ch < 4; ch++) {
    for (int dir = 1; dir >= 0; dir--) {
      uint16_t minStart = 0;
      uint16_t minRun = 0;
      float feedForward = 0.0f;
      readMotorCalibration(ch, dir == 1, &minStart, &minRun, &feedForward);
      Serial.print(names[ch]);
      Serial.print(dir == 1 ? " 正转" : " 反转");
      Serial.print(": 启动死区PWM=");
      Serial.print(minStart);
      Serial.print(" 运行死区PWM=");
      Serial.print(minRun);
      Serial.print(" 前馈=");
      Serial.println(feedForward);
    }
  }
}

// 每秒打印一次真实状态；刚启动的前几秒允许"空闲"，避免把命令还没生效误判为失败
static bool waitCalibration() {
  const char *names[] = {"空闲", "排队", "运行中", "成功", "失败"};
  uint32_t start = millis();
  uint32_t lastPrint = 0;
  while (millis() - start < kCalibrateTimeoutMs) {
    MotorCalibrationStatus status = getMotorCalibrationStatus();
    if (status == MOTOR_CALIB_SUCCESS) {
      return true;
    }
    if (status == MOTOR_CALIB_FAILED) {
      return false;
    }
    if (status == MOTOR_CALIB_IDLE && millis() - start > 8000) {
      Serial.println("状态一直为空闲，校准未启动");
      return false;
    }
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.print("[");
      Serial.print((millis() - start) / 1000);
      Serial.print("s] 校准状态=");
      Serial.println(names[(uint8_t)status]);
    }
    delay(100);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);

  Serial.println("10 秒后开始电机校准，请把小车垫起来让轮子悬空，手和线缆远离轮子");
  beep(1);
  delay(kStartDelayMs);

  uint16_t batteryMv = readBatteryMv();
  Serial.print("电池电压 mV=");
  Serial.println(batteryMv);
  if (batteryMv < kMinBatteryMv) {
    Serial.println("电池电压过低或开关在 OFF，不执行校准");
    return;
  }

  Serial.println("校准中，约 3 分钟，请勿断电");
  motorAutoCalibrateAll();
  bool ok = waitCalibration();
  if (ok) {
    Serial.println("校准成功");
    printCalibration();
    beep(2);
  } else {
    Serial.println("校准失败或超时");
    beep(3);
  }
}

void loop() {
  delay(1000);
}
