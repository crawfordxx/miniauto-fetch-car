#include "Vision.h"

#include "BoardDriver.h"
#include "Ultrasound.h"
#include "WonderLLM.h"

// 颜色编号（官方颜色识别固件）：红 0、蓝 1、绿 2。固件一次只接受一种目标颜色（同时注册三种会被拒绝），
// 模块默认就是找蓝色，可用 visionSetColor 切换。
static uint8_t s_colorTargets[] = {1};
static constexpr uint8_t kColorIdBase = 100;  // classId = 100 + 颜色编号，避免和 COCO 编号混淆

// YOLO 模式的可夹取物品清单（COCO 编号）：只保留放在地上、粗细约 5cm 以内、爪子夹得住的东西
// （实测 500ml 瓶子约 6.5cm 粗套不进爪子；书、手机、遥控器、鼠标太扁贴地，牙刷太细，已去掉）
//   瓶子类：瓶子39，以及瓶子常被认成的 花瓶75、停车收费表12、消防栓10（细长小瓶子）
//   球类：球32，以及小球常被认成的 飞盘29
//   其他小物：杯子41 香蕉46 苹果47 橙子49 胡萝卜51 甜甜圈54 玩具熊77
//   鸟14：用户摆放的可夹物品实测常被认成鸟（2026-10-12）
static constexpr uint8_t kGraspableClassIds[] = {10, 12, 14, 29, 32, 39, 41, 46, 47, 49, 51, 54, 75, 77};
static constexpr uint8_t kWonderLlmAddr = 0x54;
static constexpr uint8_t kI2cChunkSize = 120;
static constexpr uint8_t kScanTries = 3;
static constexpr uint16_t kScanGapMs = 150;
// 盯住目标时：别的类别离上次位置这么近才当作同一个物体（实测原目标漏检一帧就跳到 47 像素外的另一个物体上）
static constexpr float kSameObjectPx = 25.0f;

static wonderllm_handle_t s_vision;
static Ultrasound s_ultrasound;

static bool s_colorModeSet = false;
static bool s_lastReadOk = false;

static bool isGraspable(uint8_t classId) {
  for (uint8_t id : kGraspableClassIds) {
    if (id == classId) return true;
  }
  return false;
}

// 设置颜色识别模式；P4 可能比小车晚启动，没成功的话每次读取前再试
static bool ensureColorMode() {
  if (!kVisionUseColor || s_colorModeSet) return true;
  static uint32_t s_lastTryMs = 0;
  if (s_lastTryMs != 0 && millis() - s_lastTryMs < 1000) return false;
  s_lastTryMs = millis();
  bool ok = false;
  wonderllm_err_t ret = wonderllm_set_vision_mode(&s_vision, WONDERLLM_VISION_MODE_COLOR, s_colorTargets,
                                                  sizeof(s_colorTargets), &ok, 1000);
  s_colorModeSet = (ret == WONDERLLM_OK && ok);
  if (!s_colorModeSet) {
    // 设置被拒绝时，模块若已处于颜色识别模式（默认找蓝色），就直接用它的默认设置
    wonderllm_color_config_t config;
    if (wonderllm_get_color_config(&s_vision, &config, 1000) == WONDERLLM_OK &&
        config.mode == WONDERLLM_VISION_MODE_COLOR) {
      s_colorModeSet = true;
      Serial.println("模块已在颜色识别模式，使用默认设置");
    }
  }
  Serial.printf("设置颜色识别模式：%s（错误码=%d，应答=%d）\n", s_colorModeSet ? "成功" : "失败", (int)ret, (int)ok);
  return s_colorModeSet;
}

static wonderllm_err_t i2cWrite(const uint8_t *data, size_t len, uint32_t timeoutMs) {
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
    if (!s_ultrasound.wireWriteData(kWonderLlmAddr, &data[offset], chunk)) {
      return WONDERLLM_ERR_FAIL;
    }
    offset += chunk;
  }
  return WONDERLLM_OK;
}

static wonderllm_err_t i2cRead(uint8_t *data, size_t len, uint32_t timeoutMs) {
  if (data == nullptr || len == 0) {
    return WONDERLLM_ERR_INVALID_ARG;
  }
  if (len > 255) {
    return WONDERLLM_ERR_INVALID_SIZE;
  }
  uint32_t start = millis();
  while (timeoutMs == 0 || millis() - start < timeoutMs) {
    if (s_ultrasound.wireReadData(kWonderLlmAddr, data, len) == (int)len) {
      return WONDERLLM_OK;
    }
    delay(2);
  }
  return WONDERLLM_ERR_TIMEOUT;
}

bool visionBegin() {
  s_ultrasound.begin(EXTEND_I2C_SDA, EXTEND_I2C_SCL, 400000UL);
  if (wonderllm_init(&s_vision) != WONDERLLM_OK ||
      wonderllm_register_i2c_callbacks(&s_vision, i2cWrite, i2cRead) != WONDERLLM_OK) {
    return false;
  }
  return ensureColorMode();
}

// 统一的候选框：两种识别结果都转成这个，按规则打分，留分最高的
static void considerBox(Target &t, float &bestScore, float trackX, int trackClass, uint8_t classId, uint16_t x,
                        uint16_t y, uint16_t w, uint16_t h) {
  if (w == 0 || h == 0) return;
  float cx = x + w / 2.0f;  // x 是框左上角
  if (trackX >= 0.0f && trackClass >= 0 && classId != trackClass && fabsf(cx - trackX) > kSameObjectPx) return;
  float score = trackX >= 0.0f ? -fabsf(cx - trackX)           // 盯住：离上次位置越近越好
                               : (kImageMirrored ? -cx : cx);  // 挑选：车实际越靠右越好
  if (t.found && score <= bestScore) return;
  bestScore = score;
  t.found = true;
  t.classId = classId;
  t.centerX = cx;
  t.width = w;
  t.height = h;
  t.bottomY = y + h;
}

bool visionLastReadOk() {
  return s_lastReadOk;
}

bool visionReadTarget(Target &t, float trackX, int trackClass) {
  t = {};
  s_lastReadOk = false;
  float bestScore = 0.0f;
  if (!kVisionUseColor) {
    wonderllm_object_detection_result_t result;
    if (wonderllm_get_yolo11n_result(&s_vision, &result, 300) != WONDERLLM_OK) return false;
    s_lastReadOk = true;
    for (uint8_t i = 0; i < result.parsed_count; i++) {
      const wonderllm_object_detection_item_t &it = result.items[i];
      if (isGraspable(it.index)) considerBox(t, bestScore, trackX, trackClass, it.index, it.x, it.y, it.width, it.height);
    }
    return t.found;
  }
  if (!ensureColorMode()) return false;
  wonderllm_color_result_t result;
  if (wonderllm_get_color_result(&s_vision, &result, 300) != WONDERLLM_OK) {
    return false;
  }
  s_lastReadOk = true;
  for (uint8_t i = 0; i < result.count && i < WONDERLLM_MAX_COLOR_BLOCKS; i++) {
    const wonderllm_color_block_t &it = result.blocks[i];
    considerBox(t, bestScore, trackX, trackClass, kColorIdBase + it.index, it.x, it.y, it.width, it.height);
  }
  return t.found;
}

bool visionScan(Target &t, float trackX, int trackClass) {
  for (uint8_t i = 0; i < kScanTries; i++) {
    if (visionReadTarget(t, trackX, trackClass)) {
      return true;
    }
    delay(kScanGapMs);
  }
  return false;
}

void visionPrintAll() {
  if (!kVisionUseColor) {
    wonderllm_object_detection_result_t result;
    if (wonderllm_get_yolo11n_result(&s_vision, &result, 300) != WONDERLLM_OK) {
      Serial.println("读取失败");
      return;
    }
    Serial.printf("超声波=%umm 共%u个:", s_ultrasound.GetDistance(), result.parsed_count);
    for (uint8_t i = 0; i < result.parsed_count; i++) {
      const wonderllm_object_detection_item_t &it = result.items[i];
      Serial.printf(" [类别%u%s 中心X=%u 框=%ux%u 下边Y=%u]", it.index, isGraspable(it.index) ? "*" : "",
                    it.x + it.width / 2, it.width, it.height, it.y + it.height);
    }
    Serial.println();
    return;
  }
  if (!ensureColorMode()) {
    Serial.println("颜色识别模式未设置成功");
    return;
  }
  wonderllm_color_result_t result;
  if (wonderllm_get_color_result(&s_vision, &result, 300) != WONDERLLM_OK) {
    Serial.println("读取失败");
    return;
  }
  Serial.printf("超声波=%umm 共%u块:", s_ultrasound.GetDistance(), result.count);
  for (uint8_t i = 0; i < result.count && i < WONDERLLM_MAX_COLOR_BLOCKS; i++) {
    const wonderllm_color_block_t &it = result.blocks[i];
    Serial.printf(" [%s 中心X=%u 框=%ux%u 上边Y=%u 下边Y=%u]", visionClassName(kColorIdBase + it.index),
                  it.x + it.width / 2, it.width, it.height, it.y, it.y + it.height);
  }
  Serial.println();
}

void visionSetColor(uint8_t colorIndex) {
  s_colorTargets[0] = colorIndex;
  s_colorModeSet = false;  // 下次读取时按新颜色重新设置
}

uint16_t visionDistanceMm() {
  return s_ultrasound.GetDistance();
}

const char *visionClassName(uint8_t classId) {
  switch (classId) {
    case kColorIdBase + 0: return "红色物体";
    case kColorIdBase + 1: return "蓝色物体";
    case kColorIdBase + 2: return "绿色物体";
    case 10: return "小瓶子";
    case 12: return "瓶子(收费表)";
    case 14: return "物品(鸟)";
    case 29: return "球(飞盘)";
    case 32: return "球";
    case 39: return "瓶子";
    case 41: return "杯子";
    case 46: return "香蕉";
    case 47: return "苹果";
    case 49: return "橙子";
    case 51: return "胡萝卜";
    case 54: return "甜甜圈";
    case 75: return "花瓶";
    case 77: return "玩具熊";
    default: return "物体";
  }
}
