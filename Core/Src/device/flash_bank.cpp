#include "device/flash_bank.hpp"
#include "main.h"

namespace flash_bank {

namespace {
// RM0090 の表5（STM32F405：セクタ10 = 0x080C0000〜，セクタ11 = 0x080E0000〜，各128KB）
constexpr uint32_t BASE[COUNT] = {0x080C0000u, 0x080E0000u};
constexpr uint32_t SECTOR[COUNT] = {FLASH_SECTOR_10, FLASH_SECTOR_11};

void clearErrors() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}
} // namespace

const uint8_t* address(uint8_t bank) {
    return reinterpret_cast<const uint8_t*>(BASE[bank % COUNT]);
}

bool erase(uint8_t bank) {
    if (bank >= COUNT) return false;
    FLASH_EraseInitTypeDef e{};
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = SECTOR[bank];
    e.NbSectors = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 2.7〜3.6V（VDD 3.3V）：32bit ずつ
    uint32_t bad_sector = 0;

    HAL_FLASH_Unlock();
    clearErrors();
    bool ok = HAL_FLASHEx_Erase(&e, &bad_sector) == HAL_OK;
    HAL_FLASH_Lock();
    return ok;
}

bool program(uint8_t bank, uint32_t offset, const uint32_t* words, uint32_t count) {
    if (bank >= COUNT || offset % 4 != 0 || offset + count * 4 > SIZE) return false;
    HAL_FLASH_Unlock();
    clearErrors();
    bool ok = true;
    for (uint32_t i = 0; i < count && ok; ++i) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BASE[bank] + offset + i * 4, words[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
}

} // namespace flash_bank
