// 低矮家具 AI 寻物车：搜索 -> 对准 -> 靠近 -> 夹取 -> 原路带回
// 视觉：P4 模块离线 YOLO11n（I2C 0x54，画面 240x240）
// 运动：所有位移都用编码器定距，并记入路径日志，带回时按日志倒放

#include <Arduino.h>
#include <Wire.h>

#include "ArmControl.h"
#include "Mecanum.h"
#include "BoardDriver.h"
#include "Ultrasound.h"
#include "WonderLLM.h"

// ---- 底盘尺寸（与官方例程一致） ----
constexpr float kAxleWidthM = 0.140f;     // 与官方运动例程一致
constexpr float kAxleLengthM = 0.145f;
constexpr float kWheelDiameterM = 0.065f;

// ---- 可夹取物品清单（COCO 类别编号）：清单内的任何一种都会去夹，其余一律忽略 ----
// 球32 瓶子39 杯子41 香蕉46 苹果47 橙子49 鼠标64 遥控器65 手机67 书73 玩具熊77 牙刷79
constexpr uint8_t kGraspableClassIds[] = {32, 39, 41, 46, 47, 49, 64, 65, 67, 73, 77, 79};

static bool isGraspableClass(uint8_t classId) {
  for (uint8_t id : kGraspableClassIds) {
    if (id == classId) {
      return true;
    }
  }
  return false;
}

// ---- 视觉与对准 ----
constexpr float kImageCenterX = 120.0f;       // 画面 240x240
constexpr float kCenterDeadbandPx = 15.0f;    // 偏差小于此值视为已对准
constexpr float kRadPerPx = 0.0046f;          // 约 63 度视野 / 240 像素
constexpr float kSteerSign = -1.0f;           // 目标偏右时的转向符号，实车若转反了改成 +1
constexpr uint8_t kPollTries = 3;             // 每个朝向读几次检测结果
constexpr uint16_t kPollGapMs = 150;

// ---- 搜索与重定位 ----
constexpr float kSearchStepRad = 0.52f;       // 每次转 30 度
constexpr uint8_t kSearchSteps = 12;          // 转满一圈
constexpr uint8_t kMaxRelocations = 5;        // 一圈没找到时最多换位置几次
constexpr float kRelocMinM = 0.25f;
constexpr float kRelocMaxM = 0.45f;

// ---- 靠近与夹取 ----
constexpr float kApproachStepM = 0.05f;       // 每次前进 5cm 后重新看
constexpr uint16_t kGrabDistanceMm = 90;      // 超声波距离小于此值就夹
constexpr uint16_t kGrabHeightPx = 230;      // 框高几乎占满画面才算，只作超声波失灵时的兜底（瓶子 30cm 外框高就有 150）
constexpr uint8_t kMaxApproachSteps = 40;
constexpr uint32_t kStartDelayMs = 10000;   // 上电后等 10 秒再开始，留时间摆放或断电
// 演练开关：靠近这么多步后假装到位，用于小车悬空时测试夹取和带回。0=关闭（正式使用必须为 0）
constexpr uint8_t kDryRunArriveSteps = 0;
constexpr uint8_t kMaxLostRetries = 3;

// ---- 速度与超时 ----
constexpr float kDriveSpeedMps = 0.15f;
constexpr float kTurnSpeedRadS = 1.20f;
constexpr uint32_t kMotionTimeoutMs = 6000;
constexpr uint16_t kMotionSettleMs = 150;

// ---- 机械臂 ----
constexpr int kClawOpenAngle = ARM_CLAW_OPEN_ANGLE;
constexpr int kClawCloseAngle = ARM_CLAW_GRAB_ANGLE;
constexpr int kLiftDownAngle = ARM_HEIGHT_LOWER_ANGLE;
constexpr int kLiftUpAngle = ARM_HEIGHT_RAISED_ANGLE;
constexpr uint16_t kArmMoveTimeMs = 600;
constexpr uint16_t kArmSettleMs = 100;

constexpr uint8_t kWonderLlmAddr = 0x54;
constexpr uint8_t kI2cChunkSize = 120;
constexpr size_t kMaxPathSteps = 96;

struct Target {
  bool found;
  float centerX;
  uint16_t height;
};

// 路径日志：正数=前进米数/左转弧度，负数=后退/右转；isTurn 区分类型
struct PathStep {
  bool isTurn;
  float value;
};

static wonderllm_handle_t s_vision;
static Ultrasound ultrasound;
static PathStep s_path[kMaxPathSteps];
static size_t s_pathLen = 0;

static wonderllm_err_t appI2cWrite(const uint8_t *data, size_t len, uint32_t timeoutMs);
static wonderllm_err_t appI2cRead(uint8_t *data, size_t len, uint32_t timeoutMs);

static bool readTarget(Target &t);
static bool scanOnce(Target &t);
static bool searchWithRelocation(Target &t);
static bool approachAndGrab(Target &t);
static bool returnToStart();
static bool driveRecorded(float signedDistM);
static bool turnRecorded(float signedAngleRad);
static bool waitForMotion(bool accepted);
static void logStep(bool isTurn, float value);
static void setArm(int heightAngle, int clawAngle);
static void finish(bool ok);
static void beep(uint8_t count);

static bool s_done = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);
  ultrasound.begin(EXTEND_I2C_SDA, EXTEND_I2C_SCL, 400000UL);

  mecanumInit(kAxleWidthM, kAxleLengthM, kWheelDiameterM);
  mecanumStop(false);
  armControlInit();
  setArm(kLiftDownAngle, kClawOpenAngle);
  randomSeed(micros());

  if (wonderllm_init(&s_vision) != WONDERLLM_OK ||
      wonderllm_register_i2c_callbacks(&s_vision, appI2cWrite, appI2cRead) != WONDERLLM_OK) {
    Serial.println("视觉模块初始化失败，请检查 I2C 接线和地址 0x54");
    s_done = true;
    beep(3);
    return;
  }

  Serial.println("寻物车就绪，10 秒后开始搜索");
  beep(1);
  delay(kStartDelayMs);
}

void loop() {
  armControlUpdate();
  if (s_done) {
    delay(50);
    return;
  }

  Target target = {};
  if (!searchWithRelocation(target)) {
    Serial.println("多次换位置仍未找到目标，结束");
    finish(false);
    return;
  }
  if (!approachAndGrab(target)) {
    Serial.println("靠近或夹取失败");
    finish(false);
    return;
  }
  if (!returnToStart()) {
    Serial.println("带回途中运动超时");
    finish(false);
    return;
  }
  setArm(kLiftDownAngle, kClawOpenAngle);  // 到起点放下
  driveRecorded(-0.10f);
  finish(true);
}

// ---------------- 视觉 ----------------

static bool readTarget(Target &t) {
  t = {};
  wonderllm_object_detection_result_t result;
  wonderllm_err_t ret = wonderllm_get_yolo11n_result(&s_vision, &result, 300);
  if (ret != WONDERLLM_OK) {
    static uint32_t s_lastErrMs = 0;
    if (millis() - s_lastErrMs > 1000) {
      s_lastErrMs = millis();
      Serial.print("读取视觉模块失败，错误码=");
      Serial.println((int)ret);
    }
    return false;
  }
  static uint32_t s_lastDebugMs = 0;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < result.parsed_count; i++) {
    const wonderllm_object_detection_item_t &item = result.items[i];
    if (millis() - s_lastDebugMs > 500) {
      s_lastDebugMs = millis();
      Serial.print("识别到 类别=");
      Serial.print(item.index);
      Serial.print(" 框=");
      Serial.print(item.x);
      Serial.print(",");
      Serial.print(item.y);
      Serial.print(" ");
      Serial.print(item.width);
      Serial.print("x");
      Serial.println(item.height);
    }
    if (!isGraspableClass(item.index) || item.width == 0 || item.height == 0) {
      continue;
    }
    uint32_t area = (uint32_t)item.width * item.height;
    if (area > bestArea) {
      bestArea = area;
      t.found = true;
      t.centerX = item.x + item.width / 2.0f;  // x 是框左上角
      t.height = item.height;
    }
  }
  return t.found;
}

// 当前朝向多读几次，任意一次看到就算找到
static bool scanOnce(Target &t) {
  for (uint8_t i = 0; i < kPollTries; i++) {
    if (readTarget(t)) {
      return true;
    }
    delay(kPollGapMs);
  }
  return false;
}

// ---------------- 搜索 ----------------

static bool searchWithRelocation(Target &t) {
  for (uint8_t reloc = 0; reloc <= kMaxRelocations; reloc++) {
    for (uint8_t step = 0; step < kSearchSteps; step++) {
      if (scanOnce(t)) {
        Serial.print("发现目标，中心X=");
        Serial.println(t.centerX);
        return true;
      }
      if (!turnRecorded(kSearchStepRad)) {
        return false;
      }
    }
    if (reloc == kMaxRelocations) {
      break;
    }
    // 一圈没找到：随机转向后前进一段，换个位置再找
    float angle = (float)random(90, 181) * DEG_TO_RAD;
    if (random(0, 2) == 0) {
      angle = -angle;
    }
    float dist = kRelocMinM + (kRelocMaxM - kRelocMinM) * (random(0, 101) / 100.0f);
    Serial.print("换位置：转 ");
    Serial.print(angle * RAD_TO_DEG);
    Serial.print(" 度，前进 ");
    Serial.print(dist);
    Serial.println(" 米");
    if (!turnRecorded(angle) || !driveRecorded(dist)) {
      return false;
    }
  }
  return false;
}

// ---------------- 靠近与夹取 ----------------

static bool approachAndGrab(Target &t) {
  uint8_t lost = 0;
  for (uint8_t step = 0; step < kMaxApproachSteps; step++) {
    float err = t.centerX - kImageCenterX;
    uint16_t distMm = ultrasound.GetDistance();
    bool close = (distMm > 0 && distMm <= kGrabDistanceMm) || t.height >= kGrabHeightPx ||
                 (kDryRunArriveSteps > 0 && step >= kDryRunArriveSteps);

    if (fabsf(err) > kCenterDeadbandPx) {
      float angle = fabsf(err) * kRadPerPx * 0.8f;
      angle = constrain(angle, 0.05f, 0.5f);
      if (!turnRecorded(err > 0 ? kSteerSign * angle : -kSteerSign * angle)) {
        return false;
      }
    } else if (close) {
      Serial.println("到位，夹取");
      setArm(kLiftDownAngle, kClawOpenAngle);
      setArm(kLiftDownAngle, kClawCloseAngle);
      setArm(kLiftUpAngle, kClawCloseAngle);
      return true;
    } else if (!driveRecorded(kApproachStepM)) {
      return false;
    }

    if (!scanOnce(t)) {
      if (++lost > kMaxLostRetries) {
        Serial.println("目标丢失");
        return false;
      }
      // 目标可能刚好进入近处盲区：靠超声波判断是否已经够近
      uint16_t d = ultrasound.GetDistance();
      if (d > 0 && d <= kGrabDistanceMm) {
        t.centerX = kImageCenterX;
        t.height = kGrabHeightPx;
      }
    } else {
      lost = 0;
    }
  }
  return false;
}

// ---------------- 带回：按日志倒放 ----------------

static bool returnToStart() {
  Serial.print("带回，倒放 ");
  Serial.print(s_pathLen);
  Serial.println(" 段路径");
  for (size_t i = s_pathLen; i > 0; i--) {
    const PathStep &step = s_path[i - 1];
    // 倒放不再写日志，直接调用底层
    bool ok = step.isTurn
        ? waitForMotion(mecanumRotateAngle(step.value > 0 ? -kTurnSpeedRadS : kTurnSpeedRadS, fabsf(step.value)))
        : waitForMotion(mecanumMoveDistance(step.value > 0 ? -kDriveSpeedMps : kDriveSpeedMps, 0.0f, 0.0f, fabsf(step.value)));
    if (!ok) {
      return false;
    }
  }
  s_pathLen = 0;
  return true;
}

// ---------------- 带日志的运动 ----------------

static bool driveRecorded(float signedDistM) {
  float speed = signedDistM >= 0 ? kDriveSpeedMps : -kDriveSpeedMps;
  bool ok = waitForMotion(mecanumMoveDistance(speed, 0.0f, 0.0f, fabsf(signedDistM)));
  if (ok) {
    logStep(false, signedDistM);
  }
  return ok;
}

static bool turnRecorded(float signedAngleRad) {
  float rate = signedAngleRad >= 0 ? kTurnSpeedRadS : -kTurnSpeedRadS;
  bool ok = waitForMotion(mecanumRotateAngle(rate, fabsf(signedAngleRad)));
  if (ok) {
    logStep(true, signedAngleRad);
  }
  return ok;
}

static void logStep(bool isTurn, float value) {
  // 同类相邻动作合并，节省日志空间
  if (s_pathLen > 0 && s_path[s_pathLen - 1].isTurn == isTurn) {
    s_path[s_pathLen - 1].value += value;
    return;
  }
  if (s_pathLen < kMaxPathSteps) {
    s_path[s_pathLen++] = {isTurn, value};
  }
}

static bool waitForMotion(bool accepted) {
  if (!accepted) {
    Serial.println("运动命令未被驱动板接受");
    mecanumStop(true);
    delay(kMotionSettleMs);
    return false;
  }
  bool done = mecanumWaitDone(kMotionTimeoutMs);
  if (!done) {
    Serial.println("运动等待超时（编码器没有报告到位）");
  }
  mecanumStop(true);
  delay(kMotionSettleMs);
  return done;
}

// ---------------- 机械臂与收尾 ----------------

// 等待期间必须持续调用 armControlUpdate，舵机才会真正转动
static void pumpArm(uint32_t waitMs) {
  uint32_t start = millis();
  while (millis() - start < waitMs) {
    armControlUpdate();
    delay(5);
  }
}

static void setArm(int heightAngle, int clawAngle) {
  armControlSetHeightAngle(heightAngle, kArmMoveTimeMs);
  pumpArm((uint32_t)kArmMoveTimeMs + kArmSettleMs);
  armControlSetClawAngle(clawAngle, kArmMoveTimeMs);
  pumpArm((uint32_t)kArmMoveTimeMs + kArmSettleMs);
}

static void finish(bool ok) {
  mecanumStop(true);
  s_done = true;
  Serial.println(ok ? "寻物成功，已带回起点" : "任务中止");
  beep(ok ? 1 : 3);
}

static void beep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    setBuzzer(1200, 50);
    delay(80);
    setBuzzer(0, 0);
    delay(80);
  }
}

// ---------------- 与视觉模块的 I2C 收发（同官方例程） ----------------

static wonderllm_err_t appI2cWrite(const uint8_t *data, size_t len, uint32_t timeoutMs) {
  if (data == nullptr || len == 0) {
    return WONDERLLM_ERR_INVALID_ARG;
  }
  uint32_t start = millis();
  size_t offset = 0;
  while (offset < len) {
    if (timeoutMs > 0 && millis() - start >= timeoutMs) {
      return WONDERLLM_ERR_TIMEOUT;
    }
    size_t chunk = min((size_t)kI2cChunkSize, len - offset);
    if (!ultrasound.wireWriteData(kWonderLlmAddr, &data[offset], chunk)) {
      return WONDERLLM_ERR_FAIL;
    }
    offset += chunk;
  }
  return WONDERLLM_OK;
}

static wonderllm_err_t appI2cRead(uint8_t *data, size_t len, uint32_t timeoutMs) {
  if (data == nullptr || len == 0) {
    return WONDERLLM_ERR_INVALID_ARG;
  }
  if (len > 255) {
    return WONDERLLM_ERR_INVALID_SIZE;
  }
  uint32_t start = millis();
  while (timeoutMs == 0 || millis() - start < timeoutMs) {
    if (ultrasound.wireReadData(kWonderLlmAddr, data, len) == (int)len) {
      return WONDERLLM_OK;
    }
    delay(2);
  }
  return WONDERLLM_ERR_TIMEOUT;
}
