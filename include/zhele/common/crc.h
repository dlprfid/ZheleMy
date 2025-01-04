/**
 * @file
 * @brief Implement CRC32 module
 * @author Alezey Zhelonkin
 * @date 2024
 * @license FreeBSD
 */

#ifndef ZHELE_CRC_COMMON_H
#define ZHELE_CRC_COMMON_H

#include <stdint.h>
#include <type_traits>

namespace Zhele::Private
{
    template<typename _Clock>
    class Crc32 {
        /**
         * @brief Reset CRC unit to initial value
         * 
         * @par Returns
         *  Nothing
        */
        static void Reset();
    public:
    #if defined (CRC_POL_POL)
        enum class PolynomSize {
            Bits32 = 0b00 << CRC_CR_POLYSIZE_Pos,
            Bits16 = 0b01 << CRC_CR_POLYSIZE_Pos,
            Bits8 = 0b10 << CRC_CR_POLYSIZE_Pos,
            Bits7 = 0b11 << CRC_CR_POLYSIZE_Pos,
        };

        /**
         * @brief Set polynom size
         * 
         * @param [in] polynomSize Polynom size
         * 
         * @par Returns
         *  Nothing
        */
        static void SetPolynomSize(PolynomSize polynomSize);

        /**
         * @brief Set polynom value
         * 
         * @param [in] polynom Polynom value
         * 
         * @par Returns
         *  Nothing
        */
        static void SetPolynom(auto polynom);

        
        /**
         * @brief Calculates CRC32 checksum for given data and polynom
         * 
         * @param [in] data Data pointer
         * @param [in] size Data size
         * 
         * @returns CRC32
        */
        static uint32_t CalculateCrc32(uint32_t polynom, const uint8_t* data, unsigned size);
    #endif

    #if defined (CRC_INIT_INIT)
        /**
         * @brief Set initial value
         * 
         * @param [in] initialValue Initial value
         * 
         * @par Returns
         *  Nothing
        */
        static void SetInitialValue(uint32_t initialValue);
    #endif

        /**
         * @brief Enables CRC module clocking
         * 
         * @par Returns
         *  Nothing
        */
        static void Enable();

        /**
         * @brief Returns polynom value
         * 
         * @returns Polynom value
        */
        static uint32_t GetPolynom();

        /**
         * @brief Returns intial value
         * 
         * @returns CRC32 initial value
        */
        static uint32_t GetInitialValue();

        /**
         * @brief Calculates CRC32 checksum for given data
         * 
         * @param [in] data Data pointer
         * @param [in] size Data size
         * 
         * @returns CRC32
        */
        static uint32_t CalculateCrc32(const uint8_t* data, unsigned size);

        /**
         * @brief Store data to independet register
         * 
         * @param [in] data Data to store
         * 
         * @par Returns
         *  Nothing
        */
        static void SetIDR(std::remove_cv_t<decltype(CRC_TypeDef::IDR)> data);

        /**
         * @brief Load data from independet register
         * 
         * @returns IDR register data
        */
        static std::remove_cv_t<decltype(CRC_TypeDef::IDR)> GetIDR();
    };
}

#include "impl/crc.h"

#endif //! ZHELE_CRC_COMMON_H