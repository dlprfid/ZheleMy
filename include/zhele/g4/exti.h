/**
 * @file
 * Implements EXTI for stm32f4 series
 * 
 * @author Alexey Zhelonkin
 * @date 2020
 * @license FreeBSD
 */

#ifndef ZHELE_EXTI_H
#define ZHELE_EXTI_H

#include <stm32g4xx.h>

// For compatibility with "default" CMSIS (F0/F1/F4)
#define RTSR RTSR1
#define FTSR FTSR1
#define IMR IMR1

#include "../common/exti.h"

namespace Zhele
{
    template<uint8_t _Line, IRQn_Type _IRQn>
    template <typename port>
    void Exti<_Line, _IRQn>::SelectPort()
    {
        SYSCFG->EXTICR[_Line / 4] = (SYSCFG->EXTICR[_Line / 4] & (0xF << _Line % 4 * 4)) | ((port::Id - 'A') << (_Line % 4 * 4));
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::SelectPort(uint8_t portID)
    {
        SYSCFG->EXTICR[_Line / 4] = (SYSCFG->EXTICR[_Line / 4] & (0xF << _Line % 4 * 4)) | ((portID - 'A') << (_Line % 4 * 4));
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::EnableClock()
    {
        // Always enabled (https://community.st.com/t5/stm32-mcus-products/how-to-generate-the-clock-for-exti/m-p/347839#M87663)
    }

    using Exti0 = Exti<0, EXTI0_1_IRQn>;
    using Exti1 = Exti<1, EXTI0_1_IRQn>;

    using Exti2 = Exti<2, EXTI2_3_IRQn>;
    using Exti3 = Exti<3, EXTI2_3_IRQn>;

    using Exti4 = Exti<4, EXTI4_15_IRQn>;
    using Exti5 = Exti<5, EXTI4_15_IRQn>;
    using Exti6 = Exti<6, EXTI4_15_IRQn>;
    using Exti7 = Exti<7, EXTI4_15_IRQn>;
    using Exti8 = Exti<8, EXTI4_15_IRQn>;
    using Exti9 = Exti<9, EXTI4_15_IRQn>;
    using Exti10 = Exti<10, EXTI4_15_IRQn>;
    using Exti11 = Exti<11, EXTI4_15_IRQn>;
    using Exti12 = Exti<12, EXTI4_15_IRQn>;
    using Exti13 = Exti<13, EXTI4_15_IRQn>;
    using Exti14 = Exti<14, EXTI4_15_IRQn>;
    using Exti15 = Exti<15, EXTI4_15_IRQn>;
}
#endif //! ZHELE_EXTI_H