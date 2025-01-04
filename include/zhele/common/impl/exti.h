/**
 * @file
 * EXTI methods implementation
 * 
 * @author Alexey Zhelonkin
 * @date 2020
 * @license FreeBSD
 */
#ifndef ZHELE_EXTI_IMPL_COMMON_H
#define ZHELE_EXTI_IMPL_COMMON_H

#include <zhele/clock.h>
#include <zhele/ioports.h>
#include "zhele//common/exti.h"

namespace Zhele
{
    template<uint8_t _Line, IRQn_Type _IRQn>
    template <typename Exti<_Line, _IRQn>::Trigger trigger, typename port>
    void Exti<_Line, _IRQn>::Init()
    {
        EnableClock();
        EXTI_NS->RTSR1 = (EXTI_NS->RTSR1 & ~(1 << _Line)) | ((trigger & 1) << _Line);
        EXTI_NS->FTSR1 = (EXTI_NS->FTSR1 & ~(1 << _Line)) | (((trigger >> 1) & 1) << _Line);
        SelectPort<port>();
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::Init(Trigger trigger, char portId)
    {
        EnableClock();
        EXTI_NS->RTSR1 = (EXTI_NS->RTSR1 & ~(1 << _Line)) | ((trigger & 1) << _Line);
        EXTI_NS->FTSR1 = (EXTI_NS->FTSR1 & ~(1 << _Line)) | (((trigger >> 1) & 1) << _Line);
        SelectPort(portId);
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    template <typename pin>
    void Exti<_Line, _IRQn>::InitPin(
        typename pin::PullMode pullMode,
        typename pin::DriverType driverType,
        typename pin::Speed speed)
    {
        pin::Port::Enable();
        pin::SetConfiguration(pin::Configuration::In);
        pin::SetPullMode(pullMode);
#if !defined(STM32F1)
        pin::SetDriverType(driverType);
        pin::SetSpeed(speed);
#endif
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::EnableInterrupt()
    {
        EXTI_NS->IMR1 = (EXTI_NS->IMR1 & ~(1 << _Line)) | (1 << _Line);
        NVIC_EnableIRQ(_IRQn);
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::DisableInterrupt()
    {
        EXTI_NS->IMR1 &= (~(1 << _Line));
        NVIC_DisableIRQ(_IRQn);
    }

    template<uint8_t _Line, IRQn_Type _IRQn>
    void Exti<_Line, _IRQn>::ClearInterruptFlag()
    {
        EXTI_NS->RPR1 |= (1 << _Line);
    }
}
#endif //! ZHELE_EXTI_IMPL_COMMON_H