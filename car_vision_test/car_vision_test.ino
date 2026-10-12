// 识别测试（小车不动）：每 0.3 秒读一次 P4 的 YOLO 结果，打印物体名称、框位置、超声波距离，持续 30 秒。
#include <Arduino.h>
#include <Wire.h>

#include "BoardDriver.h"
#include "Ultrasound.h"
#include "WonderLLM.h"

constexpr uint8_t kWonderLlmAddr = 0x54;
constexpr uint8_t kI2cChunkSize = 120;
constexpr uint32_t kPollMs = 300;
constexpr uint32_t kRunMs = 30000;

static wonderllm_handle_t s_vision;
static Ultrasound ultrasound;

static const char *cocoName(uint8_t id) {
  static const char *const kNames[] = {
      "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
      "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
      "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
      "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
      "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
      "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
      "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
      "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
      "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
      "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
      "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
      "toothbrush",
  };
  return id < sizeof(kNames) / sizeof(kNames[0]) ? kNames[id] : "unknown";
}

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

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  ultrasound.begin(EXTEND_I2C_SDA, EXTEND_I2C_SCL, 400000UL);

  if (wonderllm_init(&s_vision) != WONDERLLM_OK ||
      wonderllm_register_i2c_callbacks(&s_vision, appI2cWrite, appI2cRead) != WONDERLLM_OK) {
    Serial.println("视觉模块初始化失败");
    return;
  }
  Serial.println("识别测试开始（小车不动）");

  uint32_t start = millis();
  while (millis() - start < kRunMs) {
    wonderllm_object_detection_result_t result;
    wonderllm_err_t ret = wonderllm_get_yolo11n_result(&s_vision, &result, 300);
    Serial.print("[");
    Serial.print((millis() - start) / 1000.0f, 1);
    Serial.print("s] 超声波=");
    Serial.print(ultrasound.GetDistance());
    Serial.print("mm ");
    if (ret != WONDERLLM_OK) {
      Serial.print("读取失败 错误码=");
      Serial.println((int)ret);
    } else if (result.parsed_count == 0) {
      Serial.println("没识别到东西");
    } else {
      for (uint8_t i = 0; i < result.parsed_count; i++) {
        const wonderllm_object_detection_item_t &it = result.items[i];
        Serial.print(cocoName(it.index));
        Serial.print("(");
        Serial.print(it.index);
        Serial.print(") 中心X=");
        Serial.print(it.x + it.width / 2);
        Serial.print(" 框=");
        Serial.print(it.width);
        Serial.print("x");
        Serial.print(it.height);
        Serial.print("  ");
      }
      Serial.println();
    }
    delay(kPollMs);
  }
  Serial.println("识别测试结束");
}

void loop() {
  delay(1000);
}
