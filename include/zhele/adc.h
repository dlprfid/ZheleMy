/**
 * @file
 * United header for ADC
 *
 * @author Alexey Zhelonkin
 * @date 2019
 * @licence FreeBSD
 */

#if defined(STM32F0)
    #include "f0/adc.h"
#endif
#if defined(STM32F1)
    #include "f1/adc.h"
#endif
#if defined(STM32F4)
    #include "f4/adc.h"
#endif
#if defined(STM32G0)
    #include "g0/adc.h"
#endif
#if defined(STM32G4)
    #include "g4/adc.h"
#endif
//#if defined(STM32L5)
//#include "l5/adc.h"
//#endif