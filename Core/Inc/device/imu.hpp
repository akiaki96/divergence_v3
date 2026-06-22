#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"

class Imu {
public:
    void init();
    void update();
    void calibrate();

    float accelX() const { return accelX_; }
    float accelY() const { return accelY_; }
    float accelZ() const { return accelZ_; }

    float gyroX() const { return gyroX_; }
    float gyroY() const { return gyroY_; }
    float gyroZ() const { return gyroZ_; }

    float gyroAngleX() const { return gyroAngleX_; }
    float gyroAngleY() const { return gyroAngleY_; }
    float gyroAngleZ() const { return gyroAngleZ_; }

    float velX() const { return velX_; }
    float velY() const { return velY_; }
    float velZ() const { return velZ_; }

    float posX() const { return posX_; }
    float posY() const { return posY_; }
    float posZ() const { return posZ_; }

private:
    void writeRegister(uint8_t addr, uint8_t data);
    uint8_t readRegister(uint8_t addr);
    void readBurst();
    void raw2val();
    void calibratingUpdate();

private:
    // Raw
    int16_t accelRawX_;
    int16_t accelRawY_;
    int16_t accelRawZ_;

    int16_t gyroRawX_;
    int16_t gyroRawY_;
    int16_t gyroRawZ_;

    // Reference
    int16_t accelRefX_;
    int16_t accelRefY_;
    int16_t accelRefZ_;

    int16_t gyroRefX_;
    int16_t gyroRefY_;
    int16_t gyroRefZ_;

    // Physical value
    float accelX_;
    float accelY_;
    float accelZ_;

    float gyroX_;
    float gyroY_;
    float gyroZ_;

    // Integrated value
    float gyroAngleX_;
    float gyroAngleY_;
    float gyroAngleZ_;

    float velX_;
    float velY_;
    float velZ_;

    float posX_;
    float posY_;
    float posZ_;

	int32_t sumAccelX_ = 0;
	int32_t sumAccelY_ = 0;
	int32_t sumAccelZ_ = 0;
	int32_t sumGyroX_ = 0;
	int32_t sumGyroY_ = 0;
	int32_t sumGyroZ_ = 0;
    bool calibrating_ = false;
    int32_t time_count_ = 0;

};