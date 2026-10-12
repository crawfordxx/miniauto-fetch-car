#include "Robot.h"

#include "ArmControl.h"
#include "BoardDriver.h"
#include "Mecanum.h"
#include "Vision.h"

// ---- 底盘（与官方运动例程一致） ----
static constexpr float kAxleWidthM = 0.140f;
static constexpr float kAxleLengthM = 0.145f;
static constexpr float kWheelDiameterM = 0.065f;

// ---- 速度、电量、超时 ----
static constexpr float kStepSpeedMps = 0.10f;
static constexpr float kTurnSpeedRadS = 1.0f;
static constexpr float kJoyMaxSpeedMps = 0.12f;
static constexpr float kJoyMaxTurnRadS = 1.0f;
static constexpr uint32_t kJoyTimeoutMs = 400;
static constexpr uint32_t kMotionTimeoutMs = 8000;
static constexpr uint32_t kMotionStartTimeoutMs = 300;
static constexpr uint32_t kMinRunMs = 300;
static constexpr uint16_t kMinBatteryMv = 6800;
static constexpr uint32_t kBatteryPollMs = 2000;

// ---- 寻物：搜索、对准、靠近、夹取 ----
static constexpr float kImageCenterX = kVisionFrameWidthPx / 2.0f;
static constexpr float kCenterDeadbandPx = 12.0f;   // 偏差在这以内才直线前进
static constexpr float kDriftCorrectPx = 15.0f;     // 直线靠近途中偏差超过这个就原地小转纠正（实测直走 29cm 车头右偏，偏差从 3 涨到 25）
static constexpr float kFinalAlignPx = 10.0f;       // 最后冲刺前必须对正到这个以内
static constexpr float kRadPerPx = 1.1f / kVisionFrameWidthPx;  // 约 63 度视野 / 画面宽度（实测转 10 度约移动 50 像素@320 宽）
static constexpr float kSteerSign = 1.0f;         // 实测：发"左转"时目标在画面里往左移，所以目标偏右要发"左转"
static constexpr float kSearchStepRad = 0.52f;    // 每次转 30 度
static constexpr uint8_t kSearchSteps = 12;       // 一整圈
static constexpr uint8_t kMaxRelocations = 5;
static constexpr float kRelocMinM = 0.25f;
static constexpr float kRelocMaxM = 0.45f;
static constexpr float kMaxStepCm = 10.0f;
// 底边触发：地上的物体底边到达画面这条线时，离车是固定距离（和物体大小无关）
static constexpr uint16_t kTriggerBottomY = 210;    // 底边再往下（约 218 以后）快出画面时数值会"卡住"，不能用来判断距离
static constexpr float kFinalFromTriggerCm = 17.0f;  // 从触发线再直线走这么远，物体落在爪子中间（实测：底边 223 走 8cm 差约 10cm；底边 218 后共走 24cm 会撞上瓶子；19cm 贴太近会撞倒物品，减 2cm）
static constexpr float kFinalSpeedMps = 0.06f;       // 最后冲刺放慢：撞到物体时冲击小，车头不容易被别转
static constexpr uint16_t kFarBottomY = 150;         // 底边在这条线以上（远处）每步走 kMaxStepCm
static constexpr uint16_t kNearBottomY = 185;        // 150~185 每步 kMidStepCm，再往下每步 kFineStepCm，免得一步跨过触发线（实测 Y=163 走 10cm 直接跳到 219）
static constexpr float kMidStepCm = 5.0f;
static constexpr float kFineStepCm = 3.0f;
static constexpr float kMinStepForGrowCm = 5.0f;    // 前进这么多以上，底边应明显下移
static constexpr uint16_t kMinBottomGrowPx = 5;      // 下移不到这么多像素就当作还是旧画面
static constexpr float kMaxFinalMoveCm = 40.0f;
static constexpr float kAlignTurnSpeedRadS = 0.4f;  // 对准时慢速转，减少小角度冲过头
static constexpr int kCarryLiftAngle = 70;         // 搬运时的升降角度（58 最低，132 最高）
static constexpr int kGrabClawAngle = ARM_CLAW_CLOSED_ANGLE;  // 夹取一律合到最紧（90 度=完全闭合；官方默认 102 偏松，抬起会掉）
static constexpr int kHoldBelowAngle = 125;        // 爪子角度小于这个值视为在夹东西
static constexpr uint32_t kMaxHoldMs = 30000;      // 夹持超过这么久，爪子自动卸力，防止舵机长时间堵转发热
static constexpr uint32_t kMaxCarryHoldMs = 90000; // 带回起点途中放宽：路线长时 30 秒不够，会在半路掉东西
static constexpr uint8_t kMaxGrabRetries = 2;      // 没夹住时最多重试几次
static constexpr float kVerifyBackoffM = 0.10f;    // 夹完先后退这么远再检查有没有夹住
static constexpr uint16_t kMissBottomYPx = 170;    // 夹后目标下边仍低于这条线 = 还躺在车前地上（实测 22cm 处下边约 200）
static constexpr float kFrameEdgePx = 3.0f;        // 框贴着画面边缘说明被截断了，宽度不可信，这种样本不用
static constexpr float kFrameWidthPx = kVisionFrameWidthPx;
static constexpr uint8_t kMaxApproachSteps = 60;
static constexpr uint8_t kMaxLostRetries = 5;     // 远处小目标常连续几帧漏检，多等几次
static constexpr float kSameReadingPx = 4.0f;      // 前后两次读数相差小于这个像素，视为同一帧旧画面
static constexpr uint32_t kStaleWaitMs = 1500;    // 读到和动作前一样的结果时，最多再等这么久拿新画面
static constexpr uint16_t kArmMoveTimeMs = 600;
static constexpr uint16_t kArmSettleMs = 100;
static constexpr uint16_t kVisionSettleMs = 700;   // 动完先等 P4 出新画面，避免用转之前的旧结果
static constexpr bool kBuzzerEnabled = false;      // 用户要求暂时关闭蜂鸣器
static constexpr uint32_t kInitialLookMs = 4000;   // 开始时原地多看一会儿，确实没有再转
static constexpr float kEdgeMarginPx = 40.0f;      // 离画面左右边缘这么近的目标要再确认
static constexpr float kEdgeTurnRad = 0.26f;       // 约 15 度
static constexpr uint32_t kConfirmLookMs = 2000;   // 搜索时第一次看到目标后再确认的时间
static constexpr uint32_t kSearchDwellMs = 2500;   // 搜索时每转一步停下来看的时间（用户反馈 1.5 秒太短，还没认出就转走了）
static constexpr float kYawDriftRadPerCm = 0.0026f; // 实测直走 29cm 车头顺时针偏约 4.3 度
static constexpr float kMinYawFixRad = 0.035f;     // 后退收尾时剩余偏转超过约 2 度就补
static constexpr float kMinTurnRad = 0.06f;        // 约 3.4 度（慢速转时小角度也转得动）

static constexpr uint8_t kMaxTriggerBackoffs = 3;  // 开始靠近时已越过触发线，最多后退几次退回线外
static constexpr float kSameTargetPx = 40.0f;      // 夹后自检：离原目标水平位置这么近的才算"原来那个"
static constexpr float kReturnChunkM = 0.30f;      // 返回时一次最多走这么远（3 秒），不会撞上运动超时
static constexpr float kReturnChunkRad = 1.57f;    // 返回时一次最多转 90 度
static constexpr float kMinSegmentM = 0.005f;      // 合并后几乎抵消的路段直接跳过（太小驱动板不启动，会被判超时）
static constexpr float kMinSegmentRad = 0.02f;

static constexpr size_t kMaxPathSteps = 128;

// 路线记录：前进/平移单位米（前、右为正），转向单位弧度（左为正）
enum PathKind : uint8_t { PATH_FORWARD, PATH_STRAFE, PATH_TURN };
struct PathStep {
  PathKind kind;
  float value;
};

enum RobotState : uint8_t { ST_IDLE, ST_MOVING, ST_FETCHING, ST_RETURNING };

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static Command s_pending = {CMD_NONE, 0, 0.0f};
static volatile bool s_cancel = false;
static volatile bool s_lookRequested = false;
static volatile bool s_releaseRequested = false;  // 网页/串口要求卸力：任何时候都要尽快执行
static bool s_justReleased = false;
static bool s_pathFull = false;
static volatile bool s_grabCheckEnabled = true;   // 夹完检查有没有夹住，没夹住自动重试；爪子坏了时用 check off 关掉
static volatile RobotState s_state = ST_IDLE;
static volatile uint16_t s_batteryMv = 0;
static char s_msg[96] = "就绪";

// 摇杆输入整组在 s_lock 内读写，避免读到"旧 x + 新 y"这种没人发过的组合
struct JoyInput {
  float x;
  float y;
  bool strafe;
  uint32_t ms;
};
static JoyInput s_joy = {0.0f, 0.0f, false, 0};
static bool s_joyLatched = false;  // 停止/卸力/执行指令后置位：摇杆松手回零之前不再驱动
static bool s_joyMoving = false;
static uint32_t s_lastRunMs = 0;   // 上一次运动实际跑了多久，被打断时用来估算走了多少

// 路线只在动作线程里读写；s_pathLen 供状态显示
static PathStep s_path[kMaxPathSteps];
static volatile size_t s_pathLen = 0;

// ---------------- 状态与消息 ----------------

static void setMsg(const char *text) {
  portENTER_CRITICAL(&s_lock);
  strncpy(s_msg, text, sizeof(s_msg) - 1);
  s_msg[sizeof(s_msg) - 1] = '\0';
  portEXIT_CRITICAL(&s_lock);
  Serial.println(text);
}

static void beep(uint8_t count) {
  if (!kBuzzerEnabled) return;
  for (uint8_t i = 0; i < count; i++) {
    setBuzzer(1200, 50);
    delay(80);
    setBuzzer(0, 0);
    delay(80);
  }
}

// ---------------- 可打断的等待与运动 ----------------

// 安全：夹着东西时舵机一直顶着用力会发热（曾因绕过防堵转保护导致舵机冒烟），
// 所以不做任何"刷新加力"，夹持超时只让爪子卸力（升降臂保持，不会突然掉下）。
// 本函数在所有等待循环里反复调用，所以网页按"卸力"时即使正在寻物也能马上生效。
static bool s_holding = false;
static uint32_t s_holdStartMs = 0;

static void gripSafetyCheck() {
  if (s_releaseRequested) {
    s_releaseRequested = false;
    armControlRelease();
    s_holding = false;
    s_justReleased = true;
    setMsg("舵机已卸力，可以用手掰动");
    return;
  }
  uint32_t limit = s_state == ST_RETURNING ? kMaxCarryHoldMs : kMaxHoldMs;
  if (!s_holding || millis() - s_holdStartMs < limit) return;
  s_holding = false;
  armControlReleaseClaw();
  setMsg("夹持时间过长，为保护舵机爪子已自动卸力");
}

static void pumpArm(uint32_t waitMs) {
  uint32_t start = millis();
  while (millis() - start < waitMs) {
    gripSafetyCheck();
    if (s_cancel) return;  // 停止/卸力：不再等，后面的舵机动作也都会跳过，不会把刚卸力的舵机重新上力
    armControlUpdate();
    delay(5);
  }
}

// 与官方 mecanumWaitDone 逻辑相同，但每 20ms 检查一次"停止"
static bool waitMotion(bool accepted) {
  s_lastRunMs = 0;
  if (!accepted) {
    mecanumStop(true);
    setMsg("运动命令未被驱动板接受");
    return false;
  }
  uint32_t start = millis();
  bool started = false;
  while (true) {
    gripSafetyCheck();
    armControlUpdate();
    uint32_t elapsed = millis() - start;
    s_lastRunMs = elapsed;
    if (s_cancel) {
      mecanumStop(true);
      return false;
    }
    bool done = mecanumIsMotionDone();
    if (!done) {
      started = true;
    }
    if (started && done && elapsed >= kMinRunMs) {
      mecanumStop(true);
      delay(100);
      return true;
    }
    if ((!started && elapsed >= kMotionStartTimeoutMs) || elapsed >= kMotionTimeoutMs) {
      mecanumStop(true);
      setMsg("运动超时");
      return false;
    }
    delay(20);
  }
}

// 被打断的运动按已跑时间估算完成了多少（0~1）
static float doneFraction(float amount, float speed) {
  if (fabsf(amount) < 1e-4f) return 1.0f;
  return constrain(s_lastRunMs / 1000.0f * speed / fabsf(amount), 0.0f, 1.0f);
}

// 动之前先确认记录还放得下（同类动作会合并，不占新位置）；放不下就不动，免得走了却记不下、回不到起点
static bool pathHasRoom(PathKind kind) {
  size_t len = s_pathLen;
  if (len < kMaxPathSteps || s_path[len - 1].kind == kind) return true;
  s_pathFull = true;
  s_cancel = true;
  setMsg("路线记录已满，请先返回起点");
  return false;
}

static void logStep(PathKind kind, float value) {
  if (value == 0.0f) return;
  size_t len = s_pathLen;
  if (len > 0 && s_path[len - 1].kind == kind) {
    s_path[len - 1].value += value;  // 同类相邻动作合并
    return;
  }
  if (len < kMaxPathSteps) {  // 动之前已用 pathHasRoom 预留过
    s_path[len] = {kind, value};
    s_pathLen = len + 1;
  }
}

// 已经要求停止/卸力时不再发新的运动
static bool cancelledBeforeMotion() {
  if (!s_cancel) return false;
  s_lastRunMs = 0;
  return true;
}

// 底层动作（不记录）：forwardM 前正后负，strafeM 右正左负，turnRad 左正右负
static bool rawForward(float m, float speed = kStepSpeedMps) {
  if (cancelledBeforeMotion()) return false;
  return waitMotion(mecanumMoveDistance(m >= 0 ? speed : -speed, 0.0f, 0.0f, fabsf(m)));
}
static bool rawStrafe(float m) {
  if (cancelledBeforeMotion()) return false;
  return waitMotion(mecanumMoveDistance(0.0f, m >= 0 ? kStepSpeedMps : -kStepSpeedMps, 0.0f, fabsf(m)));
}
static bool rawTurn(float rad, float speed = kTurnSpeedRadS) {
  if (cancelledBeforeMotion()) return false;
  return waitMotion(mecanumRotateAngle(rad >= 0 ? speed : -speed, fabsf(rad)));
}

// 带记录的动作：走完记整段；中途被打断按已跑时间估算记下走了的部分
static bool moveForward(float m, float speed = kStepSpeedMps) {
  if (!pathHasRoom(PATH_FORWARD)) return false;
  bool ok = rawForward(m, speed);
  logStep(PATH_FORWARD, ok ? m : m * doneFraction(m, speed));
  return ok;
}
static bool moveStrafe(float m) {
  if (!pathHasRoom(PATH_STRAFE)) return false;
  bool ok = rawStrafe(m);
  logStep(PATH_STRAFE, ok ? m : m * doneFraction(m, kStepSpeedMps));
  return ok;
}
static bool turn(float rad, float speed = kTurnSpeedRadS) {
  if (!pathHasRoom(PATH_TURN)) return false;
  bool ok = rawTurn(rad, speed);
  logStep(PATH_TURN, ok ? rad : rad * doneFraction(rad, speed));
  return ok;
}

// 只动爪子，不碰升降臂（升降臂可能已卸力，顺带设置会把它重新唤醒、自己转动）
// 已要求停止/卸力时什么都不做：卸力后旧动作链不能再给舵机上力
static void setClaw(int clawAngle) {
  if (s_cancel) return;
  clawAngle = constrain(clawAngle, ARM_CLAW_CLOSED_ANGLE, ARM_CLAW_OPEN_ANGLE);  // 绝不往闭合位置以内顶
  bool holding = clawAngle < kHoldBelowAngle;
  if (holding && !s_holding) s_holdStartMs = millis();
  s_holding = holding;
  armControlSetClawAngle(clawAngle, kArmMoveTimeMs);
  pumpArm(kArmMoveTimeMs + kArmSettleMs);
}

// 只动升降臂，不碰爪子
static void setLift(int heightAngle) {
  if (s_cancel) return;
  armControlSetHeightAngle(heightAngle, kArmMoveTimeMs);
  pumpArm(kArmMoveTimeMs + kArmSettleMs);
}

static void setArm(int heightAngle, int clawAngle) {
  setLift(heightAngle);
  setClaw(clawAngle);
}

// ---------------- 返回起点 ----------------

// 倒放最后一段中的一小块（合并后的长路段拆开走，单次不超过运动超时）；
// 走了多少（被打断时按已跑时间估算）就从记录里扣掉多少，done 返回扣掉的量
static bool returnChunk(PathStep &step, float &done) {
  bool isTurn = step.kind == PATH_TURN;
  int pieces = (int)ceilf(fabsf(step.value) / (isTurn ? kReturnChunkRad : kReturnChunkM));
  float chunk = step.value / max(pieces, 1);
  bool ok = step.kind == PATH_FORWARD ? rawForward(-chunk)
          : step.kind == PATH_STRAFE  ? rawStrafe(-chunk)
                                      : rawTurn(-chunk);
  done = ok ? chunk : chunk * doneFraction(chunk, isTurn ? kTurnSpeedRadS : kStepSpeedMps);
  step.value -= done;
  return ok;
}

static bool returnToStart() {
  char buf[64];
  snprintf(buf, sizeof(buf), "返回起点：倒放 %u 段路线", (unsigned)s_pathLen);
  setMsg(buf);
  // 这辆车走直线时车头会顺时针慢慢偏（前进往右偏、后退车尾往左甩），后退时看不到路，
  // 只能按测得的偏转率估算累积的偏转，攒够一个最小转角就原地往回（逆时针）补一下
  float yawDebtRad = 0.0f;
  while (s_pathLen > 0) {
    PathStep &step = s_path[s_pathLen - 1];
    float minSeg = step.kind == PATH_TURN ? kMinSegmentRad : kMinSegmentM;
    if (fabsf(step.value) < minSeg) {
      s_pathLen = s_pathLen - 1;
      continue;
    }
    float done = 0.0f;
    bool ok = returnChunk(step, done);
    if (step.kind == PATH_FORWARD) yawDebtRad += kYawDriftRadPerCm * fabsf(done) * 100.0f;
    if (!ok) {
      return false;  // 没走完的部分已扣掉已走的量，留在记录里，可再次"返回起点"
    }
    if (yawDebtRad >= kMinTurnRad) {
      Serial.printf("后退偏转补偿：左转 %.1f 度\n", yawDebtRad * RAD_TO_DEG);
      if (!rawTurn(yawDebtRad, kAlignTurnSpeedRadS)) return false;
      yawDebtRad = 0.0f;
    }
  }
  // 倒退完还剩一点没补的偏转，超过约 2 度也补上
  if (yawDebtRad >= kMinYawFixRad) {
    Serial.printf("后退偏转补偿（收尾）：左转 %.1f 度\n", yawDebtRad * RAD_TO_DEG);
    if (!rawTurn(yawDebtRad, kAlignTurnSpeedRadS)) return false;
  }
  s_pathFull = false;
  return true;
}

// ---------------- 寻物 ----------------

// 动作结束后等一会儿再看，拿到的才是新画面
// 位置和大小都几乎没变 = 多半是动作前缓存的旧画面（实测转 10 度后读到 262→263）
static bool sameReading(const Target &a, const Target &b) {
  return a.found && b.found && fabsf(a.centerX - b.centerX) < kSameReadingPx &&
         abs((int)a.width - (int)b.width) < (int)kSameReadingPx;
}

// prev 不为空时：读到和动作前一模一样的结果，说明 P4 还没出新画面，再等一会儿
static bool lookAfterMotion(Target &t, const Target *prev = nullptr) {
  delay(kVisionSettleMs);
  Target stale = {};
  visionReadTarget(stale);  // 丢掉第一帧，可能是动作前缓存的旧结果
  uint32_t start = millis();
  while (true) {
    if (!visionScan(t, prev ? prev->centerX : -1.0f, prev ? prev->classId : -1)) return false;
    if (prev == nullptr || !sameReading(t, *prev)) return true;
    if (millis() - start > kStaleWaitMs) return true;  // 等够了还一样，就当画面确实没变
    delay(150);
  }
}

// 原地连续看一段时间，任意一次看到就算找到
static bool lookFor(Target &t, uint32_t durationMs, float trackX = -1.0f, int trackClass = -1) {
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (s_cancel) return false;
    if (visionScan(t, trackX, trackClass)) return true;
  }
  return false;
}

// 动完停稳后，像搜索时一样多盯一会儿（往回找时用，远处小目标常漏检）
static bool settleAndLook(Target &t, float trackX = -1.0f, int trackClass = -1) {
  delay(kVisionSettleMs);
  return lookFor(t, kSearchDwellMs, trackX, trackClass);
}

static bool searchTarget(Target &t) {
  for (uint8_t reloc = 0; reloc <= kMaxRelocations; reloc++) {
    for (uint8_t step = 0; step < kSearchSteps; step++) {
      if (s_cancel) return false;
      // 每个方向都停够时间再看：P4 出结果慢，看一眼就转很容易漏掉
      if (!(reloc == 0 && step == 0)) delay(kVisionSettleMs);
      bool seen = lookFor(t, (reloc == 0 && step == 0) ? kInitialLookMs : kSearchDwellMs);
      // 看到一次先别急：原地再盯一会儿，再看到才算数，并用最新一帧的位置去对准
      if (seen) {
        Target confirm = {};
        seen = lookFor(confirm, kConfirmLookMs, t.centerX, t.classId);  // 确认刚才挑中的那一个
        if (seen) t = confirm;
        else Serial.println("刚才只闪了一下，不算，继续找");
      }
      // 在画面边缘的目标常是误识别：先往它那边小转，让它进到中间区域再确认一次
      if (seen && fabsf(t.centerX - kImageCenterX) > kImageCenterX - kEdgeMarginPx) {
        float toward = (t.centerX > kImageCenterX ? 1.0f : -1.0f) * kSteerSign * kEdgeTurnRad;
        Serial.printf("目标在画面边缘（X=%.0f），先转过去再确认\n", t.centerX);
        if (!turn(toward, kAlignTurnSpeedRadS)) return false;
        Target again = {};
        seen = settleAndLook(again, kImageCenterX, t.classId);  // 转过去后它应该在画面中间附近
        if (seen) t = again;
        else Serial.println("转过去没看到，刚才是误识别，继续找");
      }
      if (seen) {
        char buf[64];
        snprintf(buf, sizeof(buf), "发现%s，开始靠近", visionClassName(t.classId));
        setMsg(buf);
        return true;
      }
      if (!turn(kSearchStepRad)) return false;
    }
    if (reloc == kMaxRelocations) break;
    // 一圈没找到：随机转向后前进一段，换个位置再找
    float angle = (float)random(90, 181) * DEG_TO_RAD * (random(0, 2) == 0 ? 1.0f : -1.0f);
    float dist = kRelocMinM + (kRelocMaxM - kRelocMinM) * (random(0, 101) / 100.0f);
    setMsg("这一圈没找到，换个位置再找");
    if (!turn(angle) || !moveForward(dist)) return false;
  }
  setMsg("多次换位置仍未找到目标");
  return false;
}

// 最后一步：爪子放低张开，直线前进 moveCm，让目标落在两片爪子中间，再合拢抬起
static bool finalGrab(float moveCm) {
  moveCm = constrain(moveCm, 0.0f, kMaxFinalMoveCm);
  Serial.printf("最后前进 %.0f 厘米后夹取\n", moveCm);
  setMsg("到位，最后靠近并夹取");
  setArm(ARM_HEIGHT_LOWER_ANGLE, ARM_CLAW_OPEN_ANGLE);
  if (moveCm > 0.5f && !moveForward(moveCm / 100.0f, kFinalSpeedMps)) return false;
  setArm(ARM_HEIGHT_LOWER_ANGLE, kGrabClawAngle);
  setArm(kCarryLiftAngle, kGrabClawAngle);  // 只抬一点贴地兜着走：舵机会慢慢泄力，抬高了球会掉
  return !s_cancel;  // 中途被停止/卸力时舵机动作已被跳过，不算夹完
}

// 框是否完整（没被画面左右边缘截断）
static bool fullyInFrame(const Target &t) {
  float left = t.centerX - t.width / 2.0f;
  float right = t.centerX + t.width / 2.0f;
  return left > kFrameEdgePx && right < kFrameWidthPx - kFrameEdgePx;
}

// 按物体底边在画面中的位置决定这一步走多远：越靠近画面底部，步子越小，免得冲过触发线
static float stepForBottomY(uint16_t bottomY) {
  if (bottomY < kFarBottomY) return kMaxStepCm;
  if (bottomY < kNearBottomY) return kMidStepCm;
  return kFineStepCm;
}

// 动作后重新找回同一个目标：看不见先原地再看；刚转完就丢了往回找；
// 前进了底边却没往下移，多半是旧画面，再等一会儿拿新画面（免得多走一步冲过触发线）
static bool reacquire(const Target &last, float movedCm, float turnedRad, Target &seen) {
  seen = {};
  for (uint8_t lost = 0; lost <= kMaxLostRetries; lost++) {
    if (s_cancel) return false;
    if (lookAfterMotion(seen, &last)) break;
    Serial.printf("  没看到目标（连续 %u 次），原地再看\n", lost + 1);
  }
  if (!seen.found && turnedRad != 0.0f) {
    Serial.println("  转过头了？往回找");
    if (turn(-turnedRad / 2.0f, kAlignTurnSpeedRadS) && !settleAndLook(seen, last.centerX, last.classId)) {
      if (turn(-turnedRad, kAlignTurnSpeedRadS)) settleAndLook(seen, last.centerX, last.classId);
    }
  }
  // 直线前进后丢了：近处识别常一半帧漏检（实测正前方的瓶子 18 帧只认出 8 帧），先多盯一会儿，
  // 还看不到就退回刚才能看到的位置再找
  if (!seen.found && turnedRad == 0.0f && movedCm > 0.0f && !s_cancel) {
    Serial.println("  前进后丢了？多盯一会儿，不行就退回刚才的位置");
    if (!lookFor(seen, kSearchDwellMs, last.centerX, last.classId) && moveForward(-movedCm / 100.0f)) {
      settleAndLook(seen, last.centerX, last.classId);
      movedCm = 0.0f;  // 已退回原位，底边不会变大，跳过下面的"旧画面"等待
    }
  }
  if (seen.found && movedCm >= kMinStepForGrowCm && seen.bottomY < last.bottomY + kMinBottomGrowPx) {
    uint32_t waitStart = millis();
    while (millis() - waitStart < kStaleWaitMs && !s_cancel) {
      delay(200);
      Target again = {};
      if (visionReadTarget(again, last.centerX, last.classId) && again.bottomY >= last.bottomY + kMinBottomGrowPx) {
        seen = again;
        break;
      }
    }
  }
  return seen.found;
}

// 根据当前目标位置做一步：偏了就原地转，底边到触发线就冲刺夹取，否则直线前进。
// 返回 false 表示动作失败或被停止；finished=true 表示已经执行了最后的夹取。
static bool approachStep(const Target &t, bool &approaching, float &movedCm, float &turnedRad, bool &finished) {
  float err = t.centerX - kImageCenterX;
  // 先原地对准（偏差 ≤ 12 像素），对准后直线靠近；中途偏差超过 15 像素才原地小转纠正。
  // 不用横移：麦轮在地毯上横移会带着车头转，实测每次靠近都往右偏。
  float allowedErr = approaching ? kDriftCorrectPx : kCenterDeadbandPx;
  if (t.bottomY >= kTriggerBottomY) allowedErr = kFinalAlignPx;  // 冲刺前对得更正
  if (fabsf(err) > allowedErr) {
    // 实测慢速小角度转向会转到指令的 1.5~2 倍，所以只按偏差的一半去转
    float angle = constrain(fabsf(err) * kRadPerPx * 0.5f, kMinTurnRad, 0.5f);
    turnedRad = err > 0 ? kSteerSign * angle : -kSteerSign * angle;
    Serial.printf("慢速%s转 %.0f 度%s\n", turnedRad > 0 ? "左" : "右", angle * RAD_TO_DEG,
                  approaching ? "（中途纠偏）" : "（对准）");
    return turn(turnedRad, kAlignTurnSpeedRadS);
  }
  if (t.bottomY >= kTriggerBottomY) {
    Serial.println("底边到达触发线");
    finished = true;
    return finalGrab(kFinalFromTriggerCm);
  }
  approaching = true;
  movedCm = stepForBottomY(t.bottomY);
  Serial.printf("前进 %.0f 厘米\n", movedCm);
  return moveForward(movedCm / 100.0f);
}

// 冲刺距离只在"刚跨过触发线"时准确。一开始底边就已在线下（目标本来就很近，或夹取失败后重试），
// 先直线后退到线外，再正常小步靠近，免得固定冲刺距离冲过头
static bool backOffBeforeTrigger(Target &t) {
  for (uint8_t i = 0; i < kMaxTriggerBackoffs && t.bottomY >= kTriggerBottomY; i++) {
    Serial.printf("目标已越过触发线（下边Y=%u），先后退\n", t.bottomY);
    if (!moveForward(-kVerifyBackoffM)) return false;
    Target last = t;
    if (!lookAfterMotion(t, &last)) {
      if (!s_cancel) setMsg("后退后看不到目标");
      return false;
    }
  }
  return true;
}

// 靠近：先原地对准，对准后直线前进；物体底边到达触发线时离车是固定距离（和物体大小无关），
// 再直线走固定的 kFinalFromTriggerCm 就让物体落在爪子中间。
static bool approachAndGrab(Target &t) {
  if (!backOffBeforeTrigger(t)) return false;
  bool approaching = false;         // 对准完成、开始直线前进后为 true
  uint16_t maxBottomY = t.bottomY;  // 车只往前走，物体只会越来越近，底边不会往回退
  for (uint8_t step = 0; step < kMaxApproachSteps; step++) {
    if (s_cancel) return false;
    Serial.printf("[靠近 %u] %s 中心X=%.0f 偏差=%.0f 框=%ux%u 下边Y=%u -> ", step + 1,
                  visionClassName(t.classId), t.centerX, t.centerX - kImageCenterX, t.width, t.height, t.bottomY);
    float movedCm = 0.0f;
    float turnedRad = 0.0f;
    bool finished = false;
    if (!approachStep(t, approaching, movedCm, turnedRad, finished)) return false;
    if (finished) return true;

    Target seen = {};
    bool found = reacquire(t, movedCm, turnedRad, seen);
    if (s_cancel) return false;
    if (found) {
      t = seen;
      // 偶尔只认出物体上半截，底边会突然变小（实测 198 跳回 137，结果又多走了 10cm），按到过的最大值算
      if (t.bottomY < maxBottomY) t.bottomY = maxBottomY;
      else maxBottomY = t.bottomY;
      continue;
    }
    // 已经很近又正对着却看不见：多半进了近处盲区（底边已出画面），扣掉刚走的这步直接去夹
    if (t.bottomY >= kNearBottomY && fabsf(t.centerX - kImageCenterX) <= kDriftCorrectPx) {
      Serial.println("  近处盲区，直接夹取");
      return finalGrab(kFinalFromTriggerCm - movedCm);
    }
    setMsg("目标丢失");
    return false;
  }
  setMsg("靠近步数用完仍未到位");
  return false;
}

enum GrabCheck : uint8_t { GRAB_HELD, GRAB_MISSED, GRAB_UNKNOWN };

// 夹起后后退一步再看：原来那个目标还完整躺在车前地上就说明没夹住；夹住的东西跟着车走，不会出现在前方地上。
// target 是夹取前最后看到的位置：只认它附近的框，旁边别的物体不能当成"没夹住"的证据。
static GrabCheck checkGrab(const Target &target) {
  // 先后退一步：没夹住的话目标还躺在原地，后退后会重新完整出现在画面里
  if (!moveForward(-kVerifyBackoffM)) return GRAB_MISSED;  // 后退没完成就无法判断，按没夹住处理（被停止时外层会马上退出）
  bool validRead = false;
  for (uint8_t i = 0; i < 2; i++) {
    Target seen = {};
    bool found = lookAfterMotion(seen, &target);
    validRead = validRead || found || visionLastReadOk();
    // 夹着的东西紧贴车头，框会被画面下沿截断；只有完整躺在车前地上的才算没夹住
    if (found && fabsf(seen.centerX - target.centerX) <= kSameTargetPx && seen.bottomY >= kMissBottomYPx &&
        seen.bottomY < 236 && fullyInFrame(seen)) {
      Serial.printf("检查：目标还在车前地上（下边Y=%u），没夹住\n", seen.bottomY);
      return GRAB_MISSED;
    }
  }
  if (!validRead) {
    Serial.println("检查：视觉模块读取失败，无法确认是否夹住");
    return GRAB_UNKNOWN;
  }
  Serial.println("检查：车前地上已看不到目标，判断为已夹住");
  return GRAB_HELD;
}

// 夹完自己检查有没有夹住；没夹住就张开、后退一点、重新对准再夹。
// 返回 GRAB_MISSED 表示失败或被停止（提示已给出）；自检关闭或视觉读取失败时返回 GRAB_UNKNOWN
static GrabCheck grabWithRetries(Target &t) {
  for (uint8_t attempt = 0; attempt <= kMaxGrabRetries; attempt++) {
    if (attempt > 0) {
      setMsg("没夹住，后退重试");
      setArm(ARM_HEIGHT_LOWER_ANGLE, ARM_CLAW_OPEN_ANGLE);
      Target last = t;
      // 检查时已经后退过，直接重新看；盯住原来那个，不换目标。近处常一半帧漏检，看一眼没有就再多盯一会儿
      if (!lookAfterMotion(t, &last) && !lookFor(t, kSearchDwellMs, last.centerX, last.classId)) break;
    }
    if (!approachAndGrab(t)) {
      if (!s_cancel) beep(3);  // 靠近途中失败，提示已由 approachAndGrab 给出
      return GRAB_MISSED;
    }
    GrabCheck check = s_grabCheckEnabled ? checkGrab(t) : GRAB_UNKNOWN;
    if (check != GRAB_MISSED) return check;
    if (s_cancel) return GRAB_MISSED;
  }
  if (!s_cancel) {
    setMsg("多次尝试仍没夹住");
    beep(3);
  }
  return GRAB_MISSED;
}

static void runFetch(bool returnHome) {
  s_pathLen = 0;  // 当前位置记为起点
  s_pathFull = false;
  setMsg("开始寻物，当前位置已记为起点");
  setArm(ARM_HEIGHT_LOWER_ANGLE, ARM_CLAW_OPEN_ANGLE);

  Target t = {};
  if (!searchTarget(t)) {
    if (!s_cancel) beep(3);
    return;
  }
  GrabCheck check = grabWithRetries(t);
  if (check == GRAB_MISSED) return;
  if (!returnHome) {
    mecanumStop(true);
    setMsg("已夹起，停在原地（不返回、不放下）");
    return;
  }
  s_state = ST_RETURNING;
  if (!returnToStart()) {
    if (!s_cancel) beep(3);
    return;
  }
  setArm(ARM_HEIGHT_LOWER_ANGLE, ARM_CLAW_OPEN_ANGLE);  // 到起点放下，停在起点不再后退（用户反馈多退 10cm 太远）
  if (s_cancel) return;
  setMsg(check == GRAB_HELD           ? "寻物成功，已带回起点"
         : s_grabCheckEnabled         ? "已回到起点（视觉读取失败，未确认是否夹住）"
                                      : "已完成夹取动作并回到起点（自检已关闭，未确认是否夹住）");
  beep(1);
}

// ---------------- 指令执行 ----------------

static bool batteryOk() {
  s_batteryMv = readBatteryMv();
  if (s_batteryMv < kMinBatteryMv) {
    setMsg("电池电压过低或开关在 OFF，已拒绝动作");
    return false;
  }
  return true;
}

// 网页上的定距直线 / 定角转向按钮
static void runManualMove(const Command &cmd) {
  s_state = ST_MOVING;
  if (cmd.type == CMD_TURN) {
    setMsg("转向中");
    float rad = cmd.value * DEG_TO_RAD;
    turn(cmd.dir == 'l' ? rad : -rad);
    return;
  }
  setMsg("直线移动中");
  float meters = cmd.value / 100.0f;
  if (cmd.dir == 'f') moveForward(meters);
  else if (cmd.dir == 'b') moveForward(-meters);
  else if (cmd.dir == 'r') moveStrafe(meters);
  else if (cmd.dir == 'l') moveStrafe(-meters);
}

// 一条指令执行完后更新状态提示
static void finishCommand(const Command &cmd) {
  if (s_pathFull) {
    // 保留"路线记录已满"的提示
  } else if (s_cancel) {
    setMsg(s_justReleased ? "已停止，舵机已卸力" : "已停止");
  } else if (cmd.type == CMD_STEP || cmd.type == CMD_TURN) {
    setMsg("就绪");
  }
  s_justReleased = false;
  s_state = ST_IDLE;
}

// 摇杆正开着车时来了别的指令：先刹车；摇杆要松手回零后才重新生效
static void stopJoystick() {
  portENTER_CRITICAL(&s_lock);
  s_joyLatched = true;
  portEXIT_CRITICAL(&s_lock);
  if (s_joyMoving) {
    mecanumStop(true);
    s_joyMoving = false;
  }
}

// 调用前 robotTask 已在取指令的同一临界区里清掉停止标志并置为忙，之后到的停止不会被冲掉
static void execute(const Command &cmd) {
  stopJoystick();
  if (!batteryOk()) {
    s_state = ST_IDLE;
    return;
  }
  switch (cmd.type) {
    case CMD_STEP:
    case CMD_TURN:
      runManualMove(cmd);
      break;
    case CMD_FETCH:
      s_state = ST_FETCHING;
      runFetch(true);
      break;
    case CMD_GRAB:
      s_state = ST_FETCHING;
      runFetch(false);
      break;
    case CMD_CLAW:
      setClaw((int)cmd.value);
      setMsg("爪子已转到指定角度");
      break;
    case CMD_LIFT:
      setLift((int)cmd.value);
      setMsg("升降臂已转到指定角度");
      break;
    case CMD_RETURN:
      s_state = ST_RETURNING;
      if (returnToStart()) {
        setMsg("已返回起点");
      }
      break;
    default:
      break;
  }
  finishCommand(cmd);
}

static void applyJoystick() {
  uint32_t now = millis();
  portENTER_CRITICAL(&s_lock);
  JoyInput in = s_joy;
  bool centered = now - in.ms >= kJoyTimeoutMs || (in.x == 0.0f && in.y == 0.0f);
  if (centered) s_joyLatched = false;  // 松手（或超时没信号）后解除锁定
  bool latched = s_joyLatched;
  portEXIT_CRITICAL(&s_lock);
  if (centered || latched) {
    if (s_joyMoving) {
      mecanumStop(true);
      s_joyMoving = false;
    }
    return;
  }
  if (!s_joyMoving && !batteryOk()) {
    stopJoystick();
    return;
  }
  float vx = in.y * kJoyMaxSpeedMps;
  float vy = in.strafe ? in.x * kJoyMaxSpeedMps : 0.0f;   // 摇杆向右 = 向右平移
  float wz = in.strafe ? 0.0f : -in.x * kJoyMaxTurnRadS;  // 摇杆向右 = 右转
  mecanumSetVelocity(vx, vy, wz);
  s_joyMoving = true;
}

static void robotTask(void *) {
  uint32_t lastBatteryMs = 0;
  for (;;) {
    gripSafetyCheck();
    armControlUpdate();

    Command cmd = {CMD_NONE, 0, 0.0f};
    portENTER_CRITICAL(&s_lock);
    cmd = s_pending;
    s_pending.type = CMD_NONE;
    if (cmd.type != CMD_NONE) {
      // 取出指令、置为忙、清停止标志在同一临界区：取出后到的停止不会被冲掉，也不会再收新指令
      s_state = ST_MOVING;
      s_cancel = false;
    }
    portEXIT_CRITICAL(&s_lock);

    if (cmd.type != CMD_NONE) {
      execute(cmd);
    } else if (s_lookRequested && s_state == ST_IDLE) {
      s_lookRequested = false;
      stopJoystick();  // 看识别结果要阻塞约 3 秒，期间摇杆不起作用，先刹车
      for (int i = 0; i < 6; i++) {
        visionPrintAll();
        delay(500);
      }
    } else {
      applyJoystick();
    }

    if (!s_joyMoving && millis() - lastBatteryMs > kBatteryPollMs) {
      lastBatteryMs = millis();
      s_batteryMv = readBatteryMv();
    }
    delay(10);
  }
}

// ---------------- 对外接口 ----------------

void robotBegin() {
  mecanumInit(kAxleWidthM, kAxleLengthM, kWheelDiameterM);
  mecanumStop(false);
  armControlInit();  // 爪子开机停在 130 度（ARM_DEFAULT_CLAW_ANGLE），不先张到最大
  randomSeed(micros());
  if (!visionBegin()) {
    setMsg("视觉模块初始化失败，寻物不可用");
  }
  s_batteryMv = readBatteryMv();
  xTaskCreatePinnedToCore(robotTask, "robot", 8192, nullptr, 1, nullptr, 1);
}

bool robotPost(const Command &cmd) {
  bool accepted = false;
  portENTER_CRITICAL(&s_lock);
  if (s_state == ST_IDLE && s_pending.type == CMD_NONE) {
    s_pending = cmd;
    accepted = true;
  }
  portEXIT_CRITICAL(&s_lock);
  return accepted;
}

void robotSetGrabCheck(bool enabled) {
  s_grabCheckEnabled = enabled;
}

void robotStop() {
  // 同一临界区里：取消排队中的指令、打断当前动作、锁定摇杆（按住不放也不会重新开起来，
  // 要等网页发来回零或 400ms 没信号才解锁；这里不清零输入，否则清零本身就会被当成"已松手"）
  portENTER_CRITICAL(&s_lock);
  s_pending.type = CMD_NONE;
  s_cancel = true;
  s_joyLatched = true;
  portEXIT_CRITICAL(&s_lock);
}

void robotRelease() {
  robotStop();  // 先走和"停止"一样的停车路径
  s_releaseRequested = true;
}

void robotLook() {
  s_lookRequested = true;
}

void robotJoystick(float x, float y, bool strafe) {
  if (!isfinite(x) || !isfinite(y)) {  // NaN/无穷大不能进电机速度
    x = 0.0f;
    y = 0.0f;
  }
  JoyInput in = {constrain(x, -1.0f, 1.0f), constrain(y, -1.0f, 1.0f), strafe, millis()};
  portENTER_CRITICAL(&s_lock);
  s_joy = in;
  portEXIT_CRITICAL(&s_lock);
}

String robotStatusJson() {
  static const char *const kStateNames[] = {"空闲", "移动中", "寻物中", "返回中"};
  char msg[sizeof(s_msg)];
  portENTER_CRITICAL(&s_lock);
  memcpy(msg, s_msg, sizeof(msg));
  portEXIT_CRITICAL(&s_lock);

  String json = "{\"state\":\"";
  json += kStateNames[s_state];
  json += "\",\"msg\":\"";
  json += msg;
  json += "\",\"battery\":";
  json += s_batteryMv;
  json += ",\"steps\":";
  json += (unsigned)s_pathLen;
  json += "}";
  return json;
}
