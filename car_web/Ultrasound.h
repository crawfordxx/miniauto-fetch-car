#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Arduino.h>

#define EXTEND_I2C_SDA        26
#define EXTEND_I2C_SCL        27
#define ULTRASOUND_I2C_ADDR   0x77

#define DISDENCE_L    0
#define DISDENCE_H    1

#define RGB_BRIGHTNESS  50

#define RGB_WORK_MODE 2

#define RGB1_R      3
#define RGB1_G      4
#define RGB1_B      5

#define RGB2_R      6
#define RGB2_G      7
#define RGB2_B      8

#define RGB1_R_BREATHING_CYCLE      9
#define RGB1_G_BREATHING_CYCLE      10
#define RGB1_B_BREATHING_CYCLE      11

#define RGB2_R_BREATHING_CYCLE      12
#define RGB2_G_BREATHING_CYCLE      13
#define RGB2_B_BREATHING_CYCLE      14

#define RGB_WORK_SIMPLE_MODE        0
#define RGB_WORK_BREATHING_MODE     1

class Ultrasound {
  public:
    Ultrasound();

    void begin(int sda = EXTEND_I2C_SDA,
               int scl = EXTEND_I2C_SCL,
               uint32_t frequency = 400000UL);

    bool wireWriteByte(uint8_t addr, uint8_t val);
    bool wireWriteData(uint8_t addr, const uint8_t *val, unsigned int len);
    bool wireWriteDataArray(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len);

    int wireReadData(uint8_t addr, uint8_t *val, unsigned int len);
    int wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);

    void Breathing(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    void Color(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    uint16_t GetDistance();
};

#endif