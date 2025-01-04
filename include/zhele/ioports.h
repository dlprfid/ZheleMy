/**
 * @file
 * United header for ioports
 * 
 * @author Alexey Zhelonkin
 * @date 2019
 * @license FreeBSD
 */

#if defined(STM32F0)
    #include "f0/ioports.h"
#endif
#if defined(STM32F1)
    #include "f1/ioports.h"
#endif
#if defined(STM32F4)
    #include "f4/ioports.h"
#endif
#if defined(STM32L4)
    #include "l4/ioports.h"
#endif
#if defined(STM32G0)
    #include "g0/ioports.h"
#endif
#if defined(STM32G4)
    #include "g4/ioports.h"
#endif
#if defined(STM32L5)
#include "l5/ioports.h"
#endif
