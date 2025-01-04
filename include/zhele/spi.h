 /**
 * @file
 * United header for SPI
 * 
 * @author Aleksei Zhelonkin
 * @date 2019
 * @licence FreeBSD
 */
#if defined(STM32F0)
    #include "f0/spi.h"
#endif
#if defined(STM32F1)
    #include "f1/spi.h"
#endif
#if defined(STM32F4)
    #include "f4/spi.h"
#endif
#if defined(STM32L4)
    #include "l4/spi.h"
#endif
#if defined(STM32G0)
    #include "g0/spi.h"
#endif
#if defined(STM32G4)
    #include "g4/spi.h"
#endif
#if defined(STM32L5)
#include "l5/spi.h"
#endif
