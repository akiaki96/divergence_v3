#pragma once

#include <cstdint>

namespace icm42688 {

// USER BANK
inline constexpr uint8_t REG_BANK_SEL = 118;
inline constexpr uint8_t REG_USER_BANK_0 = 0;
inline constexpr uint8_t REG_USER_BANK_1 = 1;
inline constexpr uint8_t REG_USER_BANK_2 = 2;
inline constexpr uint8_t REG_USER_BANK_3 = 3;
inline constexpr uint8_t REG_USER_BANK_4 = 4;

// USER BANK0 REGISTER MAP
inline constexpr uint8_t DEVICE_CONFIG = 17;
inline constexpr uint8_t DRIVE_CONFIG = 19;
inline constexpr uint8_t INT_CONFIG = 20;
inline constexpr uint8_t FIFO_CONFIG = 22;
inline constexpr uint8_t TEMP_DATA1 = 29;
inline constexpr uint8_t TEMP_DATA0 = 30;
inline constexpr uint8_t ACCEL_DATA_X1 = 31;
inline constexpr uint8_t ACCEL_DATA_X0 = 32;
inline constexpr uint8_t ACCEL_DATA_Y1 = 33;
inline constexpr uint8_t ACCEL_DATA_Y0 = 34;
inline constexpr uint8_t ACCEL_DATA_Z1 = 35;
inline constexpr uint8_t ACCEL_DATA_Z0 = 36;
inline constexpr uint8_t GYRO_DATA_X1 = 37;
inline constexpr uint8_t GYRO_DATA_X0 = 38;
inline constexpr uint8_t GYRO_DATA_Y1 = 39;
inline constexpr uint8_t GYRO_DATA_Y0 = 40;
inline constexpr uint8_t GYRO_DATA_Z1 = 41;
inline constexpr uint8_t GYRO_DATA_Z0 = 42;
inline constexpr uint8_t TMST_FSYNCH = 43;
inline constexpr uint8_t TMST_FSYNCL = 44;
inline constexpr uint8_t INT_STATUS = 45;
inline constexpr uint8_t FIFO_COUNTH = 46;
inline constexpr uint8_t FIFO_COUNTL = 47;
inline constexpr uint8_t FIFO_DATA = 48;
inline constexpr uint8_t APEX_DATA0 = 49;
inline constexpr uint8_t APEX_DATA1 = 50;
inline constexpr uint8_t APEX_DATA2 = 51;
inline constexpr uint8_t APEX_DATA3 = 52;
inline constexpr uint8_t APEX_DATA4 = 53;
inline constexpr uint8_t APEX_DATA5 = 54;
inline constexpr uint8_t INT_STATUS2 = 55;
inline constexpr uint8_t INT_STATUS3 = 56;
inline constexpr uint8_t SIGNAL_PATH_RESET = 75;
inline constexpr uint8_t INTF_CONFIG0 = 76;
inline constexpr uint8_t INTF_CONFIG1 = 77;
inline constexpr uint8_t PWR_MGMT0 = 78;
inline constexpr uint8_t GYRO_CONFIG0 = 79;
inline constexpr uint8_t ACCEL_CONFIG0 = 80;
inline constexpr uint8_t GYRO_CONFIG1 = 81;
inline constexpr uint8_t GYRO_ACCEL_CONFIG0 = 82;
inline constexpr uint8_t ACCEL_CONFIG1 = 83;
inline constexpr uint8_t TMST_CONFIG = 84;
inline constexpr uint8_t APEX_CONFIG0 = 86;
inline constexpr uint8_t SMD_CONFIG = 87;
inline constexpr uint8_t FIFO_CONFIG1 = 95;
inline constexpr uint8_t FIFO_CONFIG2 = 96;
inline constexpr uint8_t FIFO_CONFIG3 = 97;
inline constexpr uint8_t FSYNC_CONFIG = 98;
inline constexpr uint8_t INT_CONFIG0 = 99;
inline constexpr uint8_t INT_CONFIG1 = 100;
inline constexpr uint8_t INT_SOURCE0 = 101;
inline constexpr uint8_t INT_SOURCE1 = 102;
inline constexpr uint8_t INT_SOURCE3 = 104;
inline constexpr uint8_t INT_SOURCE4 = 105;
inline constexpr uint8_t FIFO_LOST_PKT0 = 108;
inline constexpr uint8_t FIFO_LOST_PKT1 = 109;
inline constexpr uint8_t SELF_TEST_CONFIG = 112;
inline constexpr uint8_t WHO_AM_I = 117;

// USER BANK1 REGISTER MAP
inline constexpr uint8_t SENSOR_CONFIG0 = 3;
inline constexpr uint8_t GYRO_CONFIG_STATIC2 = 11;
inline constexpr uint8_t GYRO_CONFIG_STATIC3 = 12;
inline constexpr uint8_t GYRO_CONFIG_STATIC4 = 13;
inline constexpr uint8_t GYRO_CONFIG_STATIC5 = 14;
inline constexpr uint8_t GYRO_CONFIG_STATIC6 = 15;
inline constexpr uint8_t GYRO_CONFIG_STATIC7 = 16;
inline constexpr uint8_t GYRO_CONFIG_STATIC8 = 17;
inline constexpr uint8_t GYRO_CONFIG_STATIC9 = 18;
inline constexpr uint8_t GYRO_CONFIG_STATIC10 = 19;
inline constexpr uint8_t XG_ST_DATA = 95;
inline constexpr uint8_t YG_ST_DATA = 96;
inline constexpr uint8_t ZG_ST_DATA = 97;
inline constexpr uint8_t TMSTVAL0 = 98;
inline constexpr uint8_t TMSTVAL1 = 99;
inline constexpr uint8_t TMSTVAL2 = 100;
inline constexpr uint8_t INTF_CONFIG4 = 122;
inline constexpr uint8_t INTF_CONFIG5 = 123;
inline constexpr uint8_t INTF_CONFIG6 = 124;

// USER BANK2 REGISTER MAP
inline constexpr uint8_t ACCEL_CONFIG_STATIC2 = 3;
inline constexpr uint8_t ACCEL_CONFIG_STATIC3 = 4;
inline constexpr uint8_t ACCEL_CONFIG_STATIC4 = 5;
inline constexpr uint8_t XA_ST_DATA = 59;
inline constexpr uint8_t YA_ST_DATA = 60;
inline constexpr uint8_t ZA_ST_DATA = 61;

// USER BANK4 REGISTER MAP
inline constexpr uint8_t APEX_CONFIG1 = 64;
inline constexpr uint8_t APEX_CONFIG2 = 65;
inline constexpr uint8_t APEX_CONFIG3 = 66;
inline constexpr uint8_t APEX_CONFIG4 = 67;
inline constexpr uint8_t APEX_CONFIG5 = 68;
inline constexpr uint8_t APEX_CONFIG6 = 69;
inline constexpr uint8_t APEX_CONFIG7 = 70;
inline constexpr uint8_t APEX_CONFIG8 = 71;
inline constexpr uint8_t APEX_CONFIG9 = 72;
inline constexpr uint8_t ACCEL_WOM_X_THR = 74;
inline constexpr uint8_t ACCEL_WOM_Y_THR = 75;
inline constexpr uint8_t ACCEL_WOM_Z_THR = 76;
inline constexpr uint8_t INT_SOURCE6 = 77;
inline constexpr uint8_t INT_SOURCE7 = 78;
inline constexpr uint8_t INT_SOURCE8 = 79;
inline constexpr uint8_t INT_SOURCE9 = 80;
inline constexpr uint8_t INT_SOURCE10 = 81;
inline constexpr uint8_t OFFSET_USER0 = 119;
inline constexpr uint8_t OFFSET_USER1 = 120;
inline constexpr uint8_t OFFSET_USER2 = 121;
inline constexpr uint8_t OFFSET_USER3 = 122;
inline constexpr uint8_t OFFSET_USER4 = 123;
inline constexpr uint8_t OFFSET_USER5 = 124;
inline constexpr uint8_t OFFSET_USER6 = 125;
inline constexpr uint8_t OFFSET_USER7 = 126;
inline constexpr uint8_t OFFSET_USER8 = 127;
} // namespace icm42688
