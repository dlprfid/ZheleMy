 /**
 * @file
 * Implements SPI protocol for stm32l4 series
 * 
 * @author Aleksei Zhelonkin
 * @date 2020
 * @licence FreeBSD
 */

#ifndef ZHELE_SPI_H
#define ZHELE_SPI_H

#include <stm32l5xx.h>

#include "../common/spi.h"
#include "../common/template_utils/pair.h"
#include "../common/template_utils/static_array.h"

#include "afio_bind.h"
#include "dma.h"

namespace Zhele
{
    namespace Private
    {
        template<typename _Regs, typename _Clock, typename _MosiPins, typename _MisoPins, typename _ClockPins, typename _SsPins, typename _DmaTx, typename _DmaRx>
        void Spi<_Regs, _Clock, _MosiPins, _MisoPins, _ClockPins, _SsPins, _DmaTx, _DmaRx>::SelectPins(int8_t mosiPinNumber, int8_t misoPinNumber, int8_t clockPinNumber, int8_t ssPinNumber)
        {
            using Type = typename _MosiPins::DataType;

            if(mosiPinNumber != -1)
            {
                _MosiPins::Enable();
                Type maskMosi(1 << mosiPinNumber);
                _MosiPins::SetConfiguration(_MosiPins::AltFunc, maskMosi);
                _MosiPins::AltFuncNumber(GetAltFunctionNumber<_Regs>, maskMosi);
            }

            if(misoPinNumber != -1)
            {
                _MisoPins::Enable();
                Type maskMiso(1 << misoPinNumber);
                _MisoPins::SetConfiguration(_MisoPins::AltFunc, maskMiso);
                _MisoPins::AltFuncNumber(GetAltFunctionNumber<_Regs>, maskMiso);
            }

            _ClockPins::Enable();
            Type maskClock(1 << clockPinNumber);
            _ClockPins::SetConfiguration(_ClockPins::AltFunc, maskClock);
            _ClockPins::AltFuncNumber(GetAltFunctionNumber<_Regs>, maskClock);
            
            if(ssPinNumber != -1)
            {
                _SsPins::Enable();
                Type maskSs(1 << ssPinNumber);
                _SsPins::SetConfiguration(_SsPins::AltFunc, maskSs);
                _SsPins::AltFuncNumber(GetAltFunctionNumber<_Regs>, maskSs);
            }
        }

        template<typename _Regs, typename _Clock, typename _MosiPins, typename _MisoPins, typename _ClockPins, typename _SsPins, typename _DmaTx, typename _DmaRx>
        template<int8_t mosiPinNumber, int8_t misoPinNumber, int8_t clockPinNumber, int8_t ssPinNumber>
        void Spi<_Regs, _Clock, _MosiPins, _MisoPins, _ClockPins, _SsPins, _DmaTx, _DmaRx>::SelectPins()
        {
            using MosiPin = std::conditional_t<mosiPinNumber != -1, typename _MosiPins::template Pin<mosiPinNumber>, typename IO::NullPin>;
            using MisoPin = std::conditional_t<misoPinNumber != -1, typename _MisoPins::template Pin<misoPinNumber>, typename IO::NullPin>;
            using ClockPin = typename _ClockPins::template Pin<clockPinNumber>;
            using SsPin = std::conditional_t<ssPinNumber != -1, typename _SsPins::template Pin<ssPinNumber>, typename IO::NullPin>;

            constexpr auto usedPorts = TemplateUtils::TypeList<typename MosiPin::Port, typename MisoPin::Port, typename ClockPin::Port, typename SsPin::Port>::remove_duplicates();
            usedPorts.foreach([](auto port){
                port.Enable();
            });

            if constexpr(mosiPinNumber != -1)
            {
                MosiPin::template SetConfiguration<MosiPin::Port::AltFunc>();
                MosiPin::template SetDriverType<MosiPin::DriverType::PushPull>();
                MosiPin::template AltFuncNumber<GetAltFunctionNumber<_Regs>>();
            }

            if constexpr(misoPinNumber != -1)
            {
                MisoPin::template SetConfiguration<MisoPin::Port::AltFunc>();
                MisoPin::template AltFuncNumber<GetAltFunctionNumber<_Regs>>();
            }

            ClockPin::template SetConfiguration<ClockPin::Port::AltFunc>();
            ClockPin::template SetDriverType<ClockPin::DriverType::PushPull>();
            ClockPin::template AltFuncNumber<GetAltFunctionNumber<_Regs>>();

            if constexpr(ssPinNumber != -1)
            {
                SsPin::template SetConfiguration<SsPin::Port::AltFunc>();
                SsPin::template SetDriverType<SsPin::DriverType::PushPull>();
                SsPin::template AltFuncNumber<GetAltFunctionNumber<_Regs>>();
            }
        }

        template<typename _Regs, typename _Clock, typename _MosiPins, typename _MisoPins, typename _ClockPins, typename _SsPins, typename _DmaTx, typename _DmaRx>
        template<typename MosiPin, typename MisoPin, typename ClockPin, typename SsPin>
        void Spi<_Regs, _Clock, _MosiPins, _MisoPins, _ClockPins, _SsPins, _DmaTx, _DmaRx>::SelectPins()
        {
            const int8_t mosiPinIndex = !std::is_same_v<MosiPin, IO::NullPin>
                                ? _MosiPins::template IndexOf<MosiPin>
                                : -1;
            const int8_t misoPinIndex = !std::is_same_v<MisoPin, IO::NullPin>
                                ? _MisoPins::template IndexOf<MisoPin>
                                : -1;
            const int8_t clockPinIndex = _ClockPins::template IndexOf<ClockPin>;

            const int8_t ssPinIndex = !std::is_same_v<MisoPin, IO::NullPin>
                                ? _SsPins::template IndexOf<SsPin>
                                : 1;

            static_assert(mosiPinIndex >= -1);
            static_assert(misoPinIndex >= -1);
            static_assert(clockPinIndex >= 0);
            static_assert(ssPinIndex >= -1);

            SelectPins<mosiPinIndex, misoPinIndex, clockPinIndex, ssPinIndex>();
        }

        IO_STRUCT_WRAPPER(SPI1, Spi1Regs, SPI_TypeDef);
        IO_STRUCT_WRAPPER(SPI2, Spi2Regs, SPI_TypeDef);
        IO_STRUCT_WRAPPER(SPI3, Spi3Regs, SPI_TypeDef);

        using Spi1SsPins    = IO::PinList<IO::Pa4, IO::Pa15>;
        using Spi1ClockPins = IO::PinList<IO::Pa5, IO::Pb3>;
        using Spi1MisoPins  = IO::PinList<IO::Pa6, IO::Pb4>;
        using Spi1MosiPins  = IO::PinList<IO::Pa7, IO::Pb5>;
        
        using Spi2SsPins    = IO::PinList<IO::Pb12, IO::Pb9>;
        using Spi2ClockPins = IO::PinList<IO::Pb13, IO::Pb10>;
        using Spi2MisoPins  = IO::PinList<IO::Pb14, IO::Pc2>;
        using Spi2MosiPins  = IO::PinList<IO::Pb15, IO::Pc3>;

        using Spi3SsPins    = IO::PinList<IO::Pa4, IO::Pa15>;
        using Spi3ClockPins = IO::PinList<IO::Pb3, IO::Pc10>;
        using Spi3MisoPins  = IO::PinList<IO::Pb4, IO::Pc11>;
        using Spi3MosiPins  = IO::PinList<IO::Pb5, IO::Pc12>;
    }
    using Spi1 = Private::Spi<
        Private::Spi1Regs, 
        Clock::Spi1Clock, 
        Private::Spi1MosiPins, 
        Private::Spi1MisoPins,
        Private::Spi1ClockPins,
        Private::Spi1SsPins,
        Dma1Stream3Channel1,
        Dma1Stream2Channel1>;

    using Spi2 = Private::Spi<
        Private::Spi2Regs, 
        Clock::Spi2Clock, 
        Private::Spi2MosiPins, 
        Private::Spi2MisoPins,
        Private::Spi2ClockPins,
        Private::Spi2SsPins,
        Dma1Stream5Channel1,
        Dma1Stream4Channel1>;

#if defined (SPI3)
    using Spi3 = Private::Spi<
        Private::Spi3Regs, 
        Clock::Spi3Clock, 
        Private::Spi3MosiPins, 
        Private::Spi3MisoPins,
        Private::Spi3ClockPins,
        Private::Spi3SsPins,
        Dma2Stream2Channel3,
        Dma2Stream1Channel3>;
#endif
}

#endif //! ZHELE_SPI_H