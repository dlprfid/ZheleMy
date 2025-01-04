/**
 * @file
 * @brief Implement flash
 * @author Alexey Zhelonkin
 * @date 2024
 * @license FreeBSD
 */

#ifndef ZHELE_FLASH_IMPL_COMMON_H
#define ZHELE_FLASH_IMPL_COMMON_H

#include <cstdint>

namespace Zhele
{
    inline constexpr unsigned Flash::SqrtOfPowerOfTwo(uint32_t value)
    {
        unsigned result = 0;

        if (value & (value - 1))
            return 0xffffffff;

        while (value != (1 << result))
            ++result;

        return result;
    }

    inline constexpr uint32_t Flash::FlashSize()
    {
    #if defined (FLASH_SIZE)
        return FLASH_SIZE;
    #elif defined (FLASH_END)
        return FLASH_END - FLASH_BASE;
    #elif defined (FLASH_BANK2_END)
        return FLASH_BANK2_END - FLASH_BASE;
    #elif defined (FLASH_BANK1_END)
        return FLASH_BANK1_END - FLASH_BASE;
    #else
        #error "Cannot determine flash size"
    #endif
    }

    inline constexpr uint32_t Flash::PageAddress(unsigned page)
    {
        return FLASH_BASE + page * PageSize(page);
    }


#ifndef FLASH_L5
    inline bool Flash::Unlock()
    {
        static constexpr uint32_t flashKey1 = 0x45670123UL;
        static constexpr uint32_t flashKey2 = 0xCDEF89ABUL;

        FLASH->KEYR = flashKey1;
        FLASH->KEYR = flashKey2;
        
        WaitWhileBusy();

        return (FLASH->CR & FLASH_CR_LOCK) > 0;
    }

    inline void Flash::Lock()
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }

    inline bool Flash::IsLock()
    {
        return (FLASH->CR & FLASH_CR_LOCK) != 0;
    }

    inline bool Flash::WritePage(void* dst, const void* src, unsigned size)
    {
        unsigned page = AddressToPage(dst);
		uint32_t offset = reinterpret_cast<uint32_t>(dst) - PageAddress(page);

        if(page > PageCount())
            return false;
        
        if(offset + size > PageSize(page))
            return false;

		return WriteFlash(dst, src, size);
    }

    inline bool Flash::WritePage(unsigned page, const void* src, unsigned size, unsigned offset)
    {
        if(page > PageCount())
            return false;
        
        if(offset + size > PageSize(page))
            return false;

		return WriteFlash(reinterpret_cast<uint8_t*>(PageAddress(page)) + offset, src, size);
    }

    inline void Flash::WaitWhileBusy()
    {
    #if defined (FLASH_SR_BSY1)
        while(FLASH->SR & FLASH_SR_BSY1 ) continue;
    #else
        while(FLASH->SR & FLASH_SR_BSY) continue;
    #endif
    }

#else

    inline bool Flash::Unlock()
    {
        static constexpr uint32_t flashKey1 = 0x45670123UL;
        static constexpr uint32_t flashKey2 = 0xCDEF89ABUL;

        FLASH->NSKEYR = flashKey1;
        FLASH->NSKEYR = flashKey2;

        WaitWhileBusy();

        return (FLASH->NSCR & FLASH_NSCR_NSLOCK) > 0;
    }

    inline bool Flash::UnlockSec()
    {
        static constexpr uint32_t flashKey1 = 0x45670123UL;
        static constexpr uint32_t flashKey2 = 0xCDEF89ABUL;

        FLASH->SECKEYR = flashKey1;
        FLASH->SECKEYR = flashKey2;

        WaitWhileBusy();

        return (FLASH->SECCR & FLASH_SECCR_SECLOCK) > 0;
    }

    inline void Flash::Lock()
    {
#ifdef FLASH_SEC
        FLASH->SECCR |= FLASH_SECCR_SECLOCK;
#else
        FLASH->NSCR |= FLASH_NSCR_NSLOCK;
#endif
    }

    inline bool Flash::IsLock()
    {
#ifdef FLASH_SEC
        return (FLASH->SECCR & FLASH_SECCR_SECLOCK) != 0;
#else
        return (FLASH->NSCR & FLASH_NSCR_NSLOCK) != 0;
#endif
    }

    inline bool Flash::WritePage(void* dst, const void* src, unsigned size)
    {
        unsigned page = AddressToPage(dst);
        uint32_t offset = reinterpret_cast<uint32_t>(dst) - PageAddress(page);

        if(page > PageCount())
            return false;

        if(offset + size > PageSize(page))
            return false;

        return WriteFlash(dst, src, size);
    }

    inline bool Flash::WritePage(unsigned page, const void* src, unsigned size, unsigned offset)
    {
        if(page > PageCount())
            return false;

        if(offset + size > PageSize(page))
            return false;

        return WriteFlash(reinterpret_cast<uint8_t*>(PageAddress(page)) + offset, src, size);
    }

    inline void Flash::WaitWhileBusy()
    {
#ifdef FLASH_SEC
        while(FLASH->SECSR & FLASH_SECSR_SECBSY ) continue;
#else
        while(FLASH->NSSR & FLASH_NSSR_NSBSY) continue;
#endif
    }

#endif

}

#endif //! ZHELE_FLASH_IMPL_COMMON_H