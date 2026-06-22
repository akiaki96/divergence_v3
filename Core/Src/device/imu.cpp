#include "device/imu.hpp"
#include "common/debug.hpp"
#include "config/mouse_config.hpp"
#include "device/ICM42688P_Register.hpp"
#include "spi.h"

void Imu::init() {
	writeRegister(icm42688::REG_BANK_SEL, icm42688::REG_USER_BANK_0);	// バンクの切り替え
	HAL_Delay(100);

	writeRegister(icm42688::PWR_MGMT0, 0x0F);	// 加速度計とジャイロを起動
	HAL_Delay(100);

	// ジャイロの設定
	writeRegister(icm42688::GYRO_CONFIG0, 0x04);		// ジャイロのスケールを±2000deg/sに設定
	HAL_Delay(1);							// ジャイロのODRを4kHzに設定
	// 加速度計の設定
	writeRegister(icm42688::ACCEL_CONFIG0, 0x26);	// 加速度計のスケールを±8gに設定
	HAL_Delay(1);							// 加速度計のODRを1kHzに設定
	// LPFの設定
	writeRegister(icm42688::GYRO_ACCEL_CONFIG0, 0x71);	// 加速度計のLPFをODR/40に設定
	HAL_Delay(1);								// ジャイロのLPFをODR/4に設定

}

void Imu::readBurst() {
    uint8_t tx_buf[13] = {};
    uint8_t rx_buf[13] = {};

    tx_buf[0] = icm42688::ACCEL_DATA_X1 | 0x80;   // 先頭アドレス + Read
    for(int i = 1; i < 13; i++)
        tx_buf[i] = 0x00;               // ダミー送信

    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, 13, 10);

    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

    accelRawX_ = ( (int16_t)((int16_t)rx_buf[1] << 8) | rx_buf[2] );
    accelRawY_ = ( (int16_t)((int16_t)rx_buf[3] << 8) | rx_buf[4] );
    accelRawZ_ = ( (int16_t)((int16_t)rx_buf[5] << 8) | rx_buf[6] );
    gyroRawX_  = ( (int16_t)((int16_t)rx_buf[7] << 8) | rx_buf[8] );
    gyroRawY_  = ( (int16_t)((int16_t)rx_buf[9] << 8) | rx_buf[10] );
    gyroRawZ_  = ( (int16_t)((int16_t)rx_buf[11] << 8) | rx_buf[12] );
    // LOG("raw ax=%d ay=%d az=%d\r\n",
    //     accelRawX_, accelRawY_, accelRawZ_);
}

void Imu::raw2val() {
	accelX_ = config::imu::ACCEL_X_SIGN * config::imu::G * (accelX_ - accelRefX_) / config::imu::ACCEL_SENSITIVITY;
	accelY_ = config::imu::ACCEL_X_SIGN * config::imu::G * (accelY_ - accelRefY_) / config::imu::ACCEL_SENSITIVITY;
	accelZ_ = config::imu::ACCEL_X_SIGN * config::imu::G * (accelZ_ - accelRefZ_) / config::imu::ACCEL_SENSITIVITY;

    gyroX_ = (gyroRawX_ - gyroRefX_) / config::imu::GYRO_SENSITIVITY;
    gyroY_ = (gyroRawY_ - gyroRefY_) / config::imu::GYRO_SENSITIVITY;
    gyroZ_ = (gyroRawZ_ - gyroRefZ_) / config::imu::GYRO_SENSITIVITY;

    gyroAngleX_ += gyroX_ * config::control::DT_S;
    gyroAngleY_ += gyroY_ * config::control::DT_S;
    gyroAngleZ_ += gyroZ_ * config::control::DT_S;

    velX_ += accelX_ * config::control::DT_S;
    velY_ += accelY_ * config::control::DT_S;
    velZ_ += accelZ_ * config::control::DT_S;

    posX_ += velX_ * config::control::DT_S;
    posY_ += velY_ * config::control::DT_S;
    posZ_ += velZ_ * config::control::DT_S;
}

void Imu::calibratingUpdate() {
    if (calibrating_) {
        sumAccelX_ += accelRawX_;
        sumAccelY_ += accelRawY_;
        sumAccelZ_ += accelRawZ_;
        sumGyroX_ += gyroRawX_;
        sumGyroY_ += gyroRawY_;
        sumGyroZ_ += gyroRawZ_;
        time_count_++;


        if (time_count_ >= config::imu::REFFERENCE_NUM) {
            accelRefX_ = (int16_t)(sumAccelX_ / time_count_);
            accelRefY_ = (int16_t)(sumAccelY_ / time_count_);
            accelRefZ_ = (int16_t)(sumAccelZ_ / time_count_);
            gyroRefX_ = (int16_t)(sumGyroX_ / time_count_);
            gyroRefY_ = (int16_t)(sumGyroY_ / time_count_);
            gyroRefZ_ = (int16_t)(sumGyroZ_ / time_count_);
            time_count_ = 0;
            calibrating_ = false;
        }
    }
}

void Imu::update() {
    readBurst();
    raw2val();
    calibratingUpdate();
    // LOG("raw ax=%d ay=%d az=%d\r\n",accelRawX_, accelRawY_, accelRawZ_);
    // LOG("accel %d %d %d\r\n",accelRefX_, accelRefY_, accelRefZ_);
    // LOG("accel %f %f %f\r\n",accelX_, accelY_, accelZ_);
}

void Imu::calibrate() {
    time_count_ = 0;

	sumAccelX_ = 0;
	sumAccelY_ = 0;
	sumAccelZ_ = 0;
	sumGyroX_ = 0;
	sumGyroY_ = 0;
	sumGyroZ_ = 0;

    calibrating_ = true;
}

void Imu::writeRegister(uint8_t addr, uint8_t data) {
	uint8_t address = addr & 0x7f;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &address, 1, 1);
	HAL_SPI_Transmit(&hspi3, &data, 1, 1);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}


uint8_t readRegister(uint8_t addr) {
	// 送信バッファに書き込み //
	uint8_t address = addr | 0x80;
	uint8_t value;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &address, 1, 1);
	HAL_SPI_Receive(&hspi3, &value, 1, 1);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	return value;
}