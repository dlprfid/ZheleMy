/**
 * @file
 * United header for delay
 * 
 * @author Alexey Zhelonkin
 * @date 2019
 * @license FreeBSD
 */

#if defined(STM32F0)
    #include "f0/delay.h"
#endif
#if defined(STM32F1)
    #include "f1/delay.h"
#endif
#if defined(STM32F4)
    #include "f4/delay.h"
#endif
#if defined(STM32L4)
    #include "l4/delay.h"
#endif
#if defined(STM32G0)
    #include "g0/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32L5)
#include "l5/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif
#if defined(STM32G4)
    #include "g4/delay.h"
#endif















































#ifndef ZHELE_DELAY_GENERAL_H
#define ZHELE_DELAY_GENERAL_H

namespace Zhele
{
    template<unsigned long ms, unsigned long CpuFreq = F_CPU>
    void delay_ms()
    {
        delay_us<ms * 1000, CpuFreq>();
    }
}

#endif //! ZHELE_DELAY_GENERAL_H