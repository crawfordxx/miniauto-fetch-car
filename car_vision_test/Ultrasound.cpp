#include <Wire.h>
#include "Ultrasound.h"

static TwoWire ultrasoundWire(1);

Ultrasound::Ultrasound()
{
}

void Ultrasound::begin(int sda, int scl, uint32_t frequency)
{
    ultrasoundWire.begin(sda, scl, frequency);
}

bool Ultrasound::wireWriteByte(uint8_t addr, uint8_t val)
{
    ultrasoundWire.beginTransmission(addr);
    ultrasoundWire.write(val);
    return (ultrasoundWire.endTransmission() == 0);
}

bool Ultrasound::wireWriteData(uint8_t addr, const uint8_t *val, unsigned int len)
{
    if (val == NULL || len == 0) {
        return false;
    }

    ultrasoundWire.beginTransmission(addr);
    if (ultrasoundWire.write(val, len) != len) {
        ultrasoundWire.endTransmission();
        return false;
    }

    return (ultrasoundWire.endTransmission() == 0);
}

bool Ultrasound::wireWriteDataArray(uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len)
{
    if (val == NULL || len == 0) {
        return false;
    }

    ultrasoundWire.beginTransmission(addr);
    ultrasoundWire.write(reg);
    for (unsigned int i = 0; i < len; i++) {
        ultrasoundWire.write(val[i]);
    }

    return (ultrasoundWire.endTransmission() == 0);
}

int Ultrasound::wireReadData(uint8_t addr, uint8_t *val, unsigned int len)
{
    unsigned int i = 0;

    if (val == NULL || len == 0 || len > 255) {
        return -1;
    }

    ultrasoundWire.requestFrom(addr, (uint8_t)len);
    while (ultrasoundWire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = ultrasoundWire.read();
        i++;
    }

    return (int)i;
}

int Ultrasound::wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned int i = 0;

    if (val == NULL || len == 0 || len > 255) {
        return -1;
    }

    if (!wireWriteByte(addr, reg)) {
        return -1;
    }

    ultrasoundWire.requestFrom(addr, (uint8_t)len);
    while (ultrasoundWire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = ultrasoundWire.read();
        i++;
    }

    return (int)i;
}

void Ultrasound::Breathing(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2)
{
    uint8_t breathing[6];
    uint8_t value = RGB_WORK_BREATHING_MODE;

    wireWriteDataArray(ULTRASOUND_I2C_ADDR, RGB_WORK_MODE, &value, 1);
    breathing[0] = r1;
    breathing[1] = g1;
    breathing[2] = b1;
    breathing[3] = r2;
    breathing[4] = g2;
    breathing[5] = b2;
    wireWriteDataArray(ULTRASOUND_I2C_ADDR, RGB1_R_BREATHING_CYCLE, breathing, 6);
}

void Ultrasound::Color(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2)
{
    uint8_t rgb[6];
    uint8_t value = RGB_WORK_SIMPLE_MODE;

    wireWriteDataArray(ULTRASOUND_I2C_ADDR, RGB_WORK_MODE, &value, 1);
    rgb[0] = r1;
    rgb[1] = g1;
    rgb[2] = b1;
    rgb[3] = r2;
    rgb[4] = g2;
    rgb[5] = b2;
    wireWriteDataArray(ULTRASOUND_I2C_ADDR, RGB1_R, rgb, 6);
}

uint16_t Ultrasound::GetDistance()
{
    uint16_t distance = 0;

    if (wireReadDataArray(ULTRASOUND_I2C_ADDR, DISDENCE_L, (uint8_t *)&distance, 2) != 2) {
        return 0;
    }

    return distance;
}