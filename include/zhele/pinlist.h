 /**
 * @file
 * United header for pinlist
 * 
 * @author Aleksei Zhelonkin
 * @date 2019
 * @licence FreeBSD
 */

#if defined(STM32F0)
    #include "f0/pinlist.h"
#endif
#if defined(STM32F1)
    #include "f1/pinlist.h"
#endif
#if defined(STM32F4)
    #include "f4/pinlist.h"
#endif
#if defined(STM32L4)
    #include "l4/pinlist.h"
#endif
#if defined(STM32G0)
    #include "g0/pinlist.h"
#endif
#if defined(STM32G4)
    #include "g4/pinlist.h"
#endif
#if defined(STM32L5)
#include "l5/pinlist.h"
#endif
