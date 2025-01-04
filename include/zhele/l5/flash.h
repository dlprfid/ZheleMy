/**
 * @file
 * Implements FLASH for stm32l4 series
 * 
 * @author Alexey Zhelonkin
 * @date 2022
 * @license FreeBSD
 */

#ifndef ZHELE_FLASH_H
#define ZHELE_FLASH_H

#include <stm32l5xx.h>

#include "../common/flash.h"

namespace Zhele
{
    const static uint32_t MaxFlashFrequence = 24000000;
    inline void Flash::ConfigureFrequence(uint32_t frequence)
    {
        uint32_t ws = (frequence - 1) / MaxFlashFrequence;
        if(ws > 0x0F)
            ws = 0x0F;
        FLASH->ACR |= ws;
    }
}

#endif //! ZHELE_FLASH_H