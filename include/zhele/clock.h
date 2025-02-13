/**
 * @file
 * United header for clock
 * 
 * @author Alexey Zhelonkin
 * @date 2019
 * @license FreeBSD
 */
#if defined (STM32F0)
    #include "f0/clock.h"
#endif
#if defined(STM32F1)
    #include "f1/clock.h"
#endif
#if defined(STM32F4)
    #include "f4/clock.h"
#endif
#if defined(STM32L4)
    #include "l4/clock.h"
#endif
#if defined(STM32G0)
    #include "g0/clock.h"
#endif
#if defined(STM32G4)
    #include "g4/clock.h"
#endif
#if defined(STM32L5)
#include "l5/clock.h"
#endif
