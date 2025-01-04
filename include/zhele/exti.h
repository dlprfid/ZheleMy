/**
 * @file
 * United header for EXTI
 *
 * @author Alexey Zhelonkin
 * @date 2019
 * @licence FreeBSD
 */

#if defined(STM32F0)
    #include "f0/exti.h"
#endif
#if defined(STM32F1)
    #include "f1/exti.h"
#endif
#if defined(STM32F4)
    #include "f4/exti.h"
#endif
#if defined(STM32L4)
    #include "l4/exti.h"
#endif
#if defined(STM32G0)
    #include "g0/exti.h"
#endif
#if defined(STM32G4)
    #include "g4/exti.h"
#endif
#if defined(STM32L5)
#include "l5/exti.h"
#endif
