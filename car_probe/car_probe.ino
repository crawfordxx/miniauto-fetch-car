// 探测底盘驱动板校准寄存器：发校准命令后，每 0.5 秒打印命令寄存器和状态寄存器的原始值。
#include <Arduino.h>
#include <Wire.h>

#include "BoardDriver.h"

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000UL);
  delay(8000);

  Serial.print("设备ID=0x");
  Serial.print(readDeviceId(), HEX);
  Serial.print(" 固件=0x");
  Serial.println(readFirmwareVersion(), HEX);
  Serial.print("电池mV=");
  Serial.println(readBatteryMv());

  writeReg8(REG_MOTOR_CALIB, 0);
  delay(2);
  writeReg8(REG_SAFETY_KEY, SAFETY_KEY_VALUE);
  Serial.print("写入后安全密钥寄存器=0x");
  Serial.println(readReg8(REG_SAFETY_KEY), HEX);
  writeReg8(REG_MOTOR_CALIB, 0x0F);

  for (int i = 0; i < 40; i++) {
    Serial.print("[");
    Serial.print(i / 2.0f, 1);
    Serial.print("s] 命令寄存器=0x");
    Serial.print(readReg8(REG_MOTOR_CALIB), HEX);
    Serial.print(" 状态寄存器=0x");
    Serial.println(readReg8(REG_DIST_STATUS), HEX);
    delay(500);
  }
  Serial.println("探测结束");
}

void loop() {
  delay(1000);
}
