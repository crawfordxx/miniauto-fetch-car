// 小车动作线程：所有 I2C 操作（电机、舵机、视觉、电池）都只在这个线程里执行。
// 网页处理函数只调用下面的接口投递指令、读取状态，不直接碰硬件。
#pragma once
#include <Arduino.h>

enum CommandType : uint8_t {
  CMD_NONE = 0,
  CMD_STEP,     // 定距直线：dir = 'f' 前 / 'b' 后 / 'l' 左平移 / 'r' 右平移，value = 厘米
  CMD_TURN,     // 定角转向：dir = 'l' 左 / 'r' 右，value = 度
  CMD_FETCH,    // 开始寻物：清空路线，当前位置记为起点
  CMD_RETURN,   // 按路线记录倒着走回起点
  CMD_GRAB,     // 只夹不回：找到并夹起后停在原地，不返回、不放下
  CMD_CLAW,     // 调试：爪子转到 value 度（90 闭合，ARM_CLAW_OPEN_ANGLE 张开）
  CMD_LIFT,     // 调试：升降臂转到 value 度（58 放下，132 抬起）
};

struct Command {
  CommandType type;
  char dir;
  float value;
};

void robotBegin();
// 投递指令；小车正忙时返回 false（停止指令请用 robotStop）
bool robotPost(const Command &cmd);
// 立即打断当前动作并刹车、取消排队中的指令，任何时候都可调用；摇杆要松手回零后才重新生效
void robotStop();
// 两个舵机卸力（不再出力、可用手掰动）：任何时候都会执行，正在动的会先停下，不检查电池
void robotRelease();
// 调试：原地不动，打印约 3 秒的识别结果（只在空闲时有效）
void robotLook();
// 调试：开关"夹没夹住"自检（爪子坏了时关掉，否则会一直重试回不到起点）
void robotSetGrabCheck(bool enabled);
// 摇杆：x/y 范围 -1~1，strafe=true 时左右为平移；摇杆移动不计入返回路线
void robotJoystick(float x, float y, bool strafe);
// 生成状态 JSON：{state,msg,battery,steps}
String robotStatusJson();
