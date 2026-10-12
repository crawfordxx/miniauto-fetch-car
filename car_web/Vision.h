// 视觉：通过 I2C 读取 P4 模块的离线识别结果，返回画面里面积最大的目标。
// 两种识别方式，由 kVisionUseColor 切换（P4 上要烧对应的官方固件）：
//   false：YOLO11n 物体识别（WonderLLM_P4_Yolo11nDetect），只认"可夹取物品清单"里的东西，画面 320x240
//   true ：颜色识别（WonderLLM_P4_ColorRecognition），找指定颜色的色块，画面 240x240
// 实测：小蓝球用颜色识别默认阈值完全认不出（需官方调色工具），YOLO 近处能认出，所以当前用 YOLO。
#pragma once
#include <Arduino.h>

static constexpr bool kVisionUseColor = false;
// 摄像头画面左右镜像（实测：车左前方的瓶子出现在画面右侧），车实际右边 = 画面里 X 小
static constexpr bool kImageMirrored = true;

struct Target {
  bool found;
  uint8_t classId;
  float centerX;     // 画面中心为 kVisionFrameWidthPx / 2
  uint16_t width;
  uint16_t height;
  uint16_t bottomY;  // 框下边在画面中的 Y（画面高 240），越近越大
};

static constexpr float kVisionFrameWidthPx = kVisionUseColor ? 240.0f : 320.0f;
static constexpr float kVisionFrameHeightPx = 240.0f;

bool visionBegin();
// 读一次。trackX < 0：画面里有多个目标时选车实际最右边的那个（搜索是向左转的，右边的先夹）；
// trackX >= 0：选离 trackX 最近的那个（靠近途中盯住同一个，不在几个目标之间来回换）；
// 再给 trackClass >= 0 时只认同一类别的，别的类别只有离 trackX 很近（同一物体被认成别的）才算
bool visionReadTarget(Target &t, float trackX = -1.0f, int trackClass = -1);
// 最近一次读取是否和视觉模块通信成功（区分"读取失败"和"画面里确实没有目标"）
bool visionLastReadOk();
// 当前朝向连续读几次，任意一次看到就算找到
bool visionScan(Target &t, float trackX = -1.0f, int trackClass = -1);
const char *visionClassName(uint8_t classId);
uint16_t visionDistanceMm();
// 调试：打印当前画面里识别到的所有东西
void visionPrintAll();
// 颜色模式下切换要找的颜色：0 红、1 蓝、2 绿（固件一次只认一种颜色）
void visionSetColor(uint8_t colorIndex);
