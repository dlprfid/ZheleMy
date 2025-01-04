/**
 * @file
 * Clock methods implementation.
 * 
 * @author Konstantin Chizhov
 * @date 2018
 * @license FreeBSD
 */

#ifndef ZHELE_CLOCK_IMPL_COMMON_H
#define ZHELE_CLOCK_IMPL_COMMON_H

#if !defined  (HSE_VALUE) 
    #warning HSE_VALUE not defined. Will be used HSE_VALUE equals 8 MHZ.
    #define HSE_VALUE    ((uint32_t)8000000)
#endif //! HSE_VALUE

#if !defined  (HSI_VALUE)
    #warning HSI_VALUE not defined. Will be used HSI_VALUE equals 8 MHZ.
    #define HSI_VALUE    ((uint32_t)8000000)
#endif //! HSI_VALUE

namespace Zhele::Clock
{
    template <typename _Regs>
    bool ClockBase<_Regs>::EnableClockSource(unsigned turnMask, unsigned waitReadyMask)
    {
        uint32_t timeoutCounter = 0;
        _Regs::Or(turnMask);
        while(((_Regs::Get() & waitReadyMask) == 0) && (timeoutCounter < ClockStartTimeout))
        {
            timeoutCounter++;
        }
        return (_Regs::Get() & waitReadyMask) != 0;  
    }

    template <typename _Regs>
    bool ClockBase<_Regs>::DisableClockSource(unsigned turnMask, unsigned waitReadyMask)
    {
        uint32_t timeoutCounter = 0;
        _Regs::And(~turnMask);
        while(((_Regs::Get() & waitReadyMask) != 0) && (timeoutCounter < ClockStartTimeout))
        {
            timeoutCounter++;
        }
        return (_Regs::Get() & waitReadyMask) == 0;
    }

    template<typename _SrcClock, typename _PrescalerBitField>
    ClockFrequenceT BusClock<_SrcClock, _PrescalerBitField>::SrcClockFreq()
    {
        return _SrcClock::ClockFreq();
    }

    template<typename _SrcClock, typename _PrescalerBitField>
    template<typename PrescalerType>
    void BusClock<_SrcClock, _PrescalerBitField>::SetPrescaler(PrescalerType prescaler)
    {
        _PrescalerBitField::Set(static_cast<ClockFrequenceT>(prescaler));
    }

    template<typename _Reg, unsigned _Mask, typename _ClockSrc>
    void ClockControl<_Reg, _Mask, _ClockSrc>::Enable()
    {
        _Reg::Or(_Mask);
    }

    template<typename _Reg, unsigned _Mask, typename _ClockSrc>
    void ClockControl<_Reg, _Mask, _ClockSrc>::Disable()
    {
        _Reg::And(~_Mask);
    }

    inline bool HseClock::Enable()
    {
        return ClockBase::EnableClockSource(RCC_CR_HSEON, RCC_CR_HSERDY);
    }

    inline bool HseClock::Disable()
    {
        return ClockBase::DisableClockSource(RCC_CR_HSEON, RCC_CR_HSERDY);
    }

    inline bool HsiClock::Enable()
    {
        return ClockBase::EnableClockSource(RCC_CR_HSION, RCC_CR_HSIRDY);
    }

    inline bool HsiClock::Disable()
    {
        return ClockBase::DisableClockSource(RCC_CR_HSION, RCC_CR_HSIRDY);
    }

    constexpr ClockFrequenceT HseClock::SrcClockFreq() { return HSE_VALUE; }
    constexpr unsigned HseClock::GetDivider() { return 1; }
    constexpr unsigned HseClock::GetMultipler() { return 1; }
    constexpr ClockFrequenceT HseClock::ClockFreq() { return SrcClockFreq(); }


    constexpr ClockFrequenceT HsiClock::SrcClockFreq() { return HSI_VALUE; }
    constexpr unsigned HsiClock::GetDivider() { return 1; }
    constexpr unsigned HsiClock::GetMultipler() { return 1; }
    constexpr ClockFrequenceT HsiClock::ClockFreq() { return SrcClockFreq(); }

    inline ClockFrequenceT PllClock::SrcClockFreq()
    {
        return GetClockSource() == External
            ? HseClock::ClockFreq()
            : HsiClock::ClockFreq();
    }

    inline ClockFrequenceT PllClock::ClockFreq()
    {
        return SrcClockFreq() / GetDivider() * GetMultipler();
    }

    inline bool PllClock::Enable()
    {
        if (GetClockSource() == Internal)
        {
            if (!HsiClock::Enable())
                return false;
        }
        else
        {
            if (!HseClock::Enable())
                return false;
        }
        return ClockBase::EnableClockSource(RCC_CR_PLLON, RCC_CR_PLLRDY);
    }

    inline void PllClock::Disable()
    {
        ClockBase::DisableClockSource(RCC_CR_PLLON, RCC_CR_PLLRDY);
    }

#if defined (RCC_CSR_LSION)
    inline bool LsiClock::Enable()
    {
        return ClockBase::EnableClockSource(RCC_CSR_LSION, RCC_CSR_LSIRDY);
    }

    inline bool LsiClock::Disable()
    {
        return ClockBase::DisableClockSource(RCC_CSR_LSION, RCC_CSR_LSIRDY);
    }
#endif

    template<SysClock::ClockSource clockSource>
    inline SysClock::ErrorCode SysClock::SelectClockSource()
    {
        uint32_t clockStatusValue;
        uint32_t clockSelectMask;
        uint32_t resultFrequence;

        if constexpr(clockSource == Internal)
        {
            clockStatusValue = RCC_CFGR_SWS_0;
            clockSelectMask = RCC_CFGR_SW_0;
            if (!HsiClock::Enable())
                return ClockSourceFailed;
            resultFrequence = HsiClock::ClockFreq();
        }
        else if constexpr(clockSource == External)
        {
            clockStatusValue = RCC_CFGR_SWS_1;
            clockSelectMask = RCC_CFGR_SW_1;
            if (!HseClock::Enable())
                return ClockSourceFailed;
            resultFrequence = HseClock::ClockFreq();
        }
        else if constexpr(clockSource == Pll)
        {
            clockStatusValue = RCC_CFGR_SWS;
            clockSelectMask = RCC_CFGR_SW;
            if (!PllClock::Enable())
                return ClockSourceFailed;
            resultFrequence = PllClock::ClockFreq() / PllClock::GetSystemOutputDivider();
        }
        else {
            // TODO:: Commented becasuse build failed on GCC 12
            //static_assert(false, "Invalid clock source");
            return InvalidClockSource;
        }

        Flash::ConfigureFrequence(resultFrequence);

        RCC->CFGR = clockSelectMask;
        
        uint32_t timeout = 10000;
        while (((RCC->CFGR & RCC_CFGR_SWS) != clockStatusValue) && --timeout)
            ;
        if(timeout == 0)
        {
            return ClockSelectFailed;
        }
        return Success;
    }

    inline ClockFrequenceT SysClock::ClockFreq()
    {
        uint32_t clockSrc = RCC->CFGR & RCC_CFGR_SWS;
        switch (clockSrc)
        {
            case RCC_CFGR_SWS_0: return HsiClock::ClockFreq();
            case RCC_CFGR_SWS_1: return HseClock::ClockFreq();
            case RCC_CFGR_SWS: return PllClock::ClockFreq() / PllClock::GetSystemOutputDivider();
        }
        return 0;
    }

    inline ClockFrequenceT SysClock::SrcClockFreq()
    {
        return ClockFreq();
    }

#if defined (RCC_CSR_LSION)
    constexpr ClockFrequenceT LsiClock::SrcClockFreq()
    {
        return 32768;
    }

    constexpr unsigned LsiClock::GetDivider() { return 1; }

    constexpr unsigned LsiClock::GetMultipler() { return 1; }

    constexpr ClockFrequenceT LsiClock::ClockFreq()
    {
        return SrcClockFreq();
    }
#endif
}

#endif //! ZHELE_CLOCK_IMPL_COMMON_H