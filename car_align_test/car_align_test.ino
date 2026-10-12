// 转向方向测试：看到可夹取物品后只原地转向、不前进，把它转到画面正中就停。最多转 8 次。
// 每次转完重新看一次，比较目标离画面中心的偏差：偏差变小 = 方向对，变大 = 方向反了。

#include <Arduino.h>
#include <Wire.h>

#include "BoardDriver.h"
#include "Mecanum.h"
#include "Ultrasound.h"
#include "WonderLLM.h"

constexpr float kAxleWidthM = 0.140f;   // 与官方运动例程一致
constexpr float kAxleLengthM = 0.145f;
constexpr float kWheelDiameterM = 0.065f;

// 以下参数与 find_fetch 保持一致
constexpr uint8_t kGraspableClassIds[] = {32, 39, 41, 46, 47, 49, 64, 65, 67, 73, 77, 79};
constexpr float kImageCenterX = 120.0f;
constexpr float kCenterDeadbandPx = 15.0f;
constexpr float kRadPerPx = 0.0046f;
constexpr float kSteerSign = -1.0f;
constexpr float kTurnSpeedRadS = 1.20f;
constexpr uint32_t kMotionTimeoutMs = 6000;

constexpr uint8_t kMaxTurns = 8;
constexpr uint32_t kStartDelayMs = 8000;
constexpr uint8_t kWonderLlmAddr = 0x54;
constexpr uint8_t kI2cChunkSize = 120;

static wonderllm_handle_t s_vision;
static Ultrasound ultrasound;

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

static bool isGraspable(uint8_t id) {
  for (uint8_t g : kGraspableClassIds) {
    if (g == id) {
      return true;
    }
  }
  return false;
}

// 读 3 次，取面积最大的可夹取物品中心 X；读不到返回 false
static bool readTargetX(float &centerX) {
  for (uint8_t tryNo = 0; tryNo < 3; tryNo++) {
    wonderllm_object_detection_result_t result;
    if (wonderllm_get_yolo11n_result(&s_vision, &result, 300) == WONDERLLM_OK) {
      uint32_t bestArea = 0;
      for (uint8_t i = 0; i < result.parsed_count; i++) {
        const wonderllm_object_detection_item_t &it = result.items[i];
        uint32_t area = (uint32_t)it.width * it.height;
        if (isGraspable(it.index) && area > bestArea) {
          bestArea = area;
          centerX = it.x + it.width / 2.0f;
        }
      }
      if (bestArea > 0) {
        return true;
      }
    }
    delay(150);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);
  ultrasound.begin(EXTEND_I2C_SDA, EXTEND_I2C_SCL, 400000UL);
  mecanumInit(kAxleWidthM, kAxleLengthM, kWheelDiameterM);
  mecanumStop(false);

  if (wonderllm_init(&s_vision) != WONDERLLM_OK ||
      wonderllm_register_i2c_callbacks(&s_vision, appI2cWrite, appI2cRead) != WONDERLLM_OK) {
    Serial.println("视觉模块初始化失败");
    return;
  }
  Serial.println("8 秒后开始转向测试（只转不走）");
  delay(kStartDelayMs);

  uint16_t batteryMv = readBatteryMv();
  if (batteryMv < 6800) {
    Serial.print("电池电压过低或开关 OFF，mV=");
    Serial.println(batteryMv);
    return;
  }

  float x = 0.0f;
  if (!readTargetX(x)) {
    Serial.println("没看到可夹取物品，测试结束");
    return;
  }

  for (uint8_t turn = 0; turn < kMaxTurns; turn++) {
    float err = x - kImageCenterX;
    Serial.print("第");
    Serial.print(turn + 1);
    Serial.print("次 目标中心X=");
    Serial.print(x);
    Serial.print(" 偏差=");
    Serial.println(err);
    if (fabsf(err) <= kCenterDeadbandPx) {
      Serial.println("已对准，测试结束");
      return;
    }

    float angle = constrain(fabsf(err) * kRadPerPx * 0.8f, 0.05f, 0.5f);
    float signedAngle = err > 0 ? kSteerSign * angle : -kSteerSign * angle;
    Serial.print("  转向 ");
    Serial.print(signedAngle > 0 ? "左" : "右");
    Serial.print(" ");
    Serial.print(angle * RAD_TO_DEG, 1);
    Serial.println(" 度");
    bool ok = mecanumRotateAngle(signedAngle > 0 ? kTurnSpeedRadS : -kTurnSpeedRadS, angle) &&
              mecanumWaitDone(kMotionTimeoutMs);
    mecanumStop(true);
    delay(300);
    if (!ok) {
      Serial.println("转向动作超时");
      return;
    }

    float newX = 0.0f;
    if (!readTargetX(newX)) {
      Serial.println("转完后看不到目标了（可能转过头或转反了），测试结束");
      return;
    }
    float newErr = newX - kImageCenterX;
    Serial.println(fabsf(newErr) < fabsf(err) ? "  偏差变小：方向正确" : "  偏差变大：方向反了！");
    x = newX;
  }
  Serial.println("转满 8 次仍未对准，测试结束");
}

void loop() {
  delay(1000);
}
