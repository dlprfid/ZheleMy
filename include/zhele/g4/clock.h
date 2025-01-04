/**
 * @file
 * Implemets clocks for stm32g0 series
 * 
 * @author Alexey Zhelonkin
 * @date 2024
 * @license FreeBSD
 */

#ifndef ZHELE_CLOCK_H
#define ZHELE_CLOCK_H

#include <stm32g4xx.h>


// For compatibility with "default" CMSIS (G4!!!)
#define RCC_CFGR_SW_HSI                    0x00000001U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000002U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000003U                         /*!< PLL selected as system clock */

#define RCC_CFGR_SWS_HSI                   0x00000004U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000008U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x0000000CU                         /*!< PLL used as system clock                   */

#include "../common/clock.h"

namespace Zhele::Clock
{
    DECLARE_IO_BITFIELD_WRAPPER(RCC->PLLCFGR, PllM, RCC_PLLCFGR_PLLM);
    DECLARE_IO_BITFIELD_WRAPPER(RCC->PLLCFGR, PllN, RCC_PLLCFGR_PLLN);
    DECLARE_IO_BITFIELD_WRAPPER(RCC->PLLCFGR, PllP, RCC_PLLCFGR_PLLP);

#if defined (RCC_PLLCFGR_PLLQ_Pos)
    DECLARE_IO_BITFIELD_WRAPPER(RCC->PLLCFGR, PllQ, RCC_PLLCFGR_PLLQ);
#endif

#if defined (RCC_PLLCFGR_PLLR_Pos)
    DECLARE_IO_BITFIELD_WRAPPER(RCC->PLLCFGR, PllR, RCC_PLLCFGR_PLLR);
#endif

    inline unsigned PllClock::GetDivider()
    {
        return PllM::Get();
    }

    template<unsigned divider>
    inline void PllClock::SetDivider()
    {
        static_assert(2 <= divider && divider <= PllM::MaxValue, "Invalide divider value");
        PllM::Set(divider);
    }

    inline unsigned PllClock::GetMultipler()
    {
        return PllN::Get();
    }

    template<unsigned multiplier>
    inline void PllClock::SetMultiplier()
    {
        static_assert(2 <= multiplier && multiplier <= 432, "Invalide multiplier value");
        PllN::Set(multiplier);
    }

    template<PllClock::ClockSource clockSource>
    inline void PllClock::SelectClockSource()
    {
        RCC->PLLCFGR = clockSource == External
                       ? RCC->PLLCFGR  | RCC_PLLCFGR_PLLSRC_HSE
                       : RCC->PLLCFGR  | RCC_PLLCFGR_PLLSRC_HSI;
    }

    inline PllClock::ClockSource PllClock::GetClockSource()
    {
        // return RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE
        //     ? ClockSource::External
        //     : ClockSource::Internal;

        switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE)
        {
            case RCC_PLLCFGR_PLLSRC_HSI:
                return ClockSource::Internal;
                break;

            case RCC_PLLCFGR_PLLSRC_HSE:
                return ClockSource::External;
                break;

            default:
                return ClockSource::Internal;
                break;
        }
    }

    inline unsigned PllClock::GetSystemOutputDivider()
    {
        return (PllR::Get() >> 2) + 2;
    }

    template<unsigned divider>
    inline void PllClock::SetSystemOutputDivider()
    {
        static_assert(divider == 2 || divider == 4 || divider == 6 || divider == 8, "Divider can be one of 2, 4, 6, 8");
        static constexpr uint8_t pllpValue = (divider - 2) << 2;
        PllR::Set(pllpValue);
    }

    inline unsigned PllClock::GetUsbOutputDivider()
    {
        return PllQ::Get();
    }

    template<unsigned divider>
    inline void PllClock::SetUsbOutputDivider()
    {
        static_assert(2 <= divider && divider <= PllQ::MaxValue, "Invalide divider value");
        PllQ::Set(divider);
    }

// #if defined (RCC_PLLCFGR_PLLQ_Pos)
//     inline unsigned PllClock::GetUsbOutputDivider()
//     {
//         return PllQ::Get();
//     }

//     template<unsigned divider>
//     inline void PllClock::SetUsbOutputDivider()
//     {
//     //   static_assert(2 <= multiplier && multiplier <= (PllQ::MaxValue + 1), "Invalid divider value!");
//        static_assert(2 <= divider && divider <= (PllQ::MaxValue + 1), "Invalid divider value!");
//        PllQ::Set(divider);
//     }
// #endif

#if defined (RCC_PLLCFGR_PLLR_Pos)
    inline unsigned PllClock::GetI2SOutputDivider()
    {
        return PllP::Get() + 1;
    }

    template<unsigned divider>
    inline void PllClock::SetI2SOutputDivider()
    {
        static_assert(2 <= divider && divider <= (PllP::MaxValue + 1), "Invalid divider value!");
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
        PllP::Set(divider - 1);
    }
#endif

    const static unsigned AhbPrescalerBitFieldOffset = RCC_CFGR_HPRE_Pos;
    const static unsigned AhbPrescalerBitFieldLength = GetBitFieldLength<(RCC_CFGR_HPRE_Msk >> RCC_CFGR_HPRE_Pos)>;
    IO_BITFIELD_WRAPPER(RCC->CFGR, AhbPrescalerBitField, uint32_t, AhbPrescalerBitFieldOffset, AhbPrescalerBitFieldLength);

    class AhbClock : public BusClock<SysClock, AhbPrescalerBitField>
    {
        using Base = BusClock<SysClock, AhbPrescalerBitField>;
    public:
        // AHB prescaler values
        enum Prescaler
        {
            Div1 = 0b0000 >> AhbPrescalerBitFieldOffset, ///< No divide (prescaler = 1)
            Div2 = 0b1000 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 2
            Div4 = 0b1001 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 4
            Div8 = 0b1010 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 8
            Div16 = 0b1011 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 16
            Div64 = 0b1100 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 64
            Div128 = 0b1101 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 128
            Div256 = 0b1110 >> AhbPrescalerBitFieldOffset, ///< Prescaler = 256
            Div512 = 0b1111 >> AhbPrescalerBitFieldOffset ///< Prescaler = 512
        };

        static ClockFrequenceT ClockFreq()
        {
            static constexpr uint8_t clockPrescShift[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

            ClockFrequenceT clock = SysClock::ClockFreq();
            uint8_t shiftBits = clockPrescShift[AhbPrescalerBitField::Get()];
            clock >>= shiftBits;
            return clock;
        }

        static void SetPrescaler(Prescaler prescaler)
        {
            Base::SetPrescaler(prescaler);
        }
    };

    DECLARE_IO_BITFIELD_WRAPPER(RCC->CFGR, ApbPrescalerBitField, RCC_CFGR_HPRE);

    /**
     * @brief Implements APB clock
     */
    class ApbClock : BusClock<AhbClock, ApbPrescalerBitField>
    {
        using Base = BusClock<AhbClock, ApbPrescalerBitField>;
    public:
        /**
         * @brief APB1 clock prescalers
         */
        enum Prescaler
        {
            Div1 = 0b000, ///< No divide (prescaler = 1)
            Div2 = 0b100, ///< Prescaler = 2
            Div4 = 0b101, ///< Prescaler = 4
            Div8 = 0b110, ///< Prescaler = 8
            Div16 = 0b111, ///< Prescaler = 16
        };

        static ClockFrequenceT ClockFreq()
        {
            static constexpr uint8_t clockPrescShift[] = {0, 0, 0, 0, 1, 2, 3, 4};

            ClockFrequenceT clock = AhbClock::ClockFreq();
            uint8_t shiftBits = clockPrescShift[ApbPrescalerBitField::Get()];
            clock >>= shiftBits;
            return clock;
        }

        template<Prescaler prescaler>
        static void SetPrescaler()
        {
            Base::SetPrescaler(prescaler);
        }
    };
    using Apb1Clock = ApbClock;
    using Apb2Clock = ApbClock;


    DECLARE_IO_BITFIELD_WRAPPER(ADC12_COMMON->CCR, AdcPrescalerBitField, ADC_CCR_PRESC);
    DECLARE_IO_BITFIELD_WRAPPER(ADC12_COMMON->CCR, AdcClockmodeBitField, ADC_CCR_CKMODE);

    class AdcClockSource : _AdcClockSource
    {
    public:
        /**
         * @brief ADC clock sources
         */
        enum ClockSource
        {
            Apb2 = 0, ///< APB2
        };

        /**
         * @brief ADC prescaler
         */
        enum Prescaler
        {
            Div0 = ADC_CCR_PRESC_0 >> AdcPrescalerBitFieldOffset, ///< Prescaler = 2
            Div2 = ADC_CCR_PRESC_1 >> AdcPrescalerBitFieldOffset, ///< Prescaler = 4
            Div4 = ADC_CCR_PRESC_2 >> AdcPrescalerBitFieldOffset, ///< Prescaler = 6
            Div8 = ADC_CCR_PRESC_3 >> AdcPrescalerBitFieldOffset, ///< Prescaler = 8
        };

        /**
         * @brief ADC clock mode
         */
        enum Clockmode
        {
            Mod0 = ADC_CCR_CKMODE_0 >> AdcClockmodeBitFieldOffset, ///< Prescaler = 0
            Mod1 = ADC_CCR_CKMODE_1 >> AdcClockmodeBitFieldOffset, ///< Prescaler = 1
        };

        /**
         * @brief Select clock source for ADC (like Belarus presidential election)
         *
         * @retval true Always
         */
        template<ClockSource source = ClockSource::Apb2>
        static bool SelectClockSource()
        {
            return true;
        }

        /**
         * @brief Set prescaler for ADC
         *
         * @par Returns
         *	Nothing
        */
        template<Prescaler prescaller>
        static void SetPrescaler()
        {
            AdcPrescalerBitField::Set((uint32_t)prescaller);
        }

        /**
         * @brief Returns source clock frequence of ADC
         *
         * @returns Clock frequence
         */
        static ClockFrequenceT SrcClockFreq()
        {
            return Apb2Clock::ClockFreq();
        }

        /**
         * @brief Returns current clock frequence of ADC
         *
         * @returns Current frequence
         */
        static ClockFrequenceT ClockFreq()
        {
            return SrcClockFreq() / ((AdcPrescalerBitField::Get() + 1) * 2);
        }
    };



    IO_REG_WRAPPER(RCC->AHB1ENR, AhbClockEnableReg, uint32_t);
    IO_REG_WRAPPER(RCC->APB1ENR1, PeriphClockEnable1, uint32_t);
    IO_REG_WRAPPER(RCC->APB1ENR2, PeriphClockEnable2, uint32_t);
    IO_REG_WRAPPER(RCC->APB2ENR,  PeriphClockEnable3, uint32_t);
    IO_REG_WRAPPER(RCC->AHB2ENR, IOPeriphClockEnable, uint32_t);

    IO_REG_WRAPPER(RCC->AHB1RSTR, AhbResetReg, uint32_t);
    IO_REG_WRAPPER(RCC->APB1RSTR1, ApbResetReg1, uint32_t);
    IO_REG_WRAPPER(RCC->APB1RSTR2, ApbResetReg2, uint32_t);

    using PortaClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOAEN, ApbClock>;
    using PortbClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOBEN, ApbClock>;
    using PortcClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOCEN, ApbClock>;
    using PortdClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIODEN, ApbClock>;
    using PorteClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOEEN, ApbClock>;
    using PortfClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOFEN, ApbClock>;
    using PortgClock = ClockControl<IOPeriphClockEnable, RCC_AHB2ENR_GPIOGEN, ApbClock>;

    using Adc1Clock = ClockControl<PeriphClockEnable2, RCC_AHB2ENR_ADC12EN, AdcClockSource>;

    using DmaClock = ClockControl<AhbClockEnableReg, RCC_AHB1ENR_DMA1EN, AhbClock>;
    using Dma1Clock = DmaClock;
    using FlashClock = ClockControl<AhbClockEnableReg, RCC_AHB1ENR_FLASHEN, AhbClock>;
    using CrcClock = ClockControl<AhbClockEnableReg, RCC_AHB1ENR_CRCEN, AhbClock>;

    using Tim3Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_TIM3EN, ApbClock>;
    using RtcClock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_RTCAPBEN, ApbClock>;
    using WatchDogClock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_WWDGEN, ApbClock>;
    using Spi2Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_SPI2EN, ApbClock>;
    using Usart2Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_USART2EN, ApbClock>;
    using I2c1Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_I2C1EN, ApbClock>;
    using I2c2Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_I2C2EN, ApbClock>;
    // using DbgClock = ClockControl<PeriphClockEnable1, RCC_APBENR1_DBGEN, ApbClock>;
    using PowerClock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_PWREN, ApbClock>;
    using SysCfgClock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_SYSCFGEN, ApbClock>;
    using Tim1Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_TIM1EN, ApbClock>;
    //  using Tim2Clock = ClockControl<PeriphClockEnable2, RCC_APB1ENR1_TIM2EN, ApbClock>;
    using Tim3Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_TIM3EN, ApbClock>;
    using Tim4Clock = ClockControl<PeriphClockEnable2, RCC_APB1ENR1_TIM4EN, ApbClock>;

    using Spi1Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_SPI1EN, ApbClock>;
    using Usart1Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_USART1EN, ApbClock>;
    using Tim15Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_TIM15EN, ApbClock>;
    using Tim16Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_TIM16EN, ApbClock>;
    using Tim17Clock = ClockControl<PeriphClockEnable3, RCC_APB2ENR_TIM17EN, ApbClock>;
    using AdcClock = ClockControl<PeriphClockEnable2, RCC_AHB2ENR_ADC12EN, AhbClock>;
//    using AdcClock2 = ClockControl<PeriphClockEnable2, RCC_AHB2ENR_ADC345EN, AhbClock>;

#if defined (RCC_APB1ENR1_TIM2EN)
    using Tim2Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_TIM2EN, ApbClock>;
#endif
#if defined (RCC_APB1ENR2_LPUART1EN)
    using LpUart1Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR2_LPUART1EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_LPTIM2EN)
    using LpTim2Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_LPTIM2EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_LPTIM1EN)
    using LpTim1Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_LPTIM1EN, ApbClock>;
#endif
#if defined (RCC_AHBENR_AESEN)
    using AesClock = ClockControl<AhbClockEnableReg, RCC_AHBENR_AESEN, AhbClock>;
#endif
#if defined (RCC_AHBENR_RNGEN)
    using RngClock = ClockControl<AhbClockEnableReg, RCC_AHBENR_RNGEN, AhbClock>;
#endif
#if defined (RCC_APBENR1_TIM6EN)
    using Tim6Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_TIM6EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_TIM7EN)
    using Tim7Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_TIM7EN, ApbClock>;
#endif
#if defined (RCC_APBENR2_TIM15EN)
    using Tim15Clock = ClockControl<PeriphClockEnable2, RCC_APBENR2_TIM15EN, ApbClock>;
#endif
#if defined (RCC_AHB2ENR_DAC1EN)
    using DacClock = ClockControl<PeriphClockEnable1, RCC_AHB2ENR_DAC1EN, ApbClock>;
#endif
#if defined (RCC_APB1ENR1_USART3EN)
    using Usart3Clock = ClockControl<PeriphClockEnable1, RCC_APB1ENR1_USART3EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_USART4EN)
    using Usart4Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_USART4EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_CECEN)
    using CecClock = ClockControl<PeriphClockEnable1, RCC_APBENR1_CECEN, ApbClock>;
#endif
#if defined (RCC_APBENR1_UCPD1EN)
    using Ucpd1Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_UCPD1EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_UCPD2EN)
    using Ucpd2Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_UCPD2EN, ApbClock>;
#endif
#if defined (RCC_IOPENR_GPIOEEN)
    using PorteClock = ClockControl<IOPeriphClockEnable, RCC_IOPENR_GPIOEEN, ApbClock>;
#endif
#if defined (RCC_AHBENR_DMA2EN)
    using Dma2Clock = ClockControl<AhbClockEnableReg, RCC_AHBENR_DMA2EN, AhbClock>;
#endif
#if defined (RCC_APBENR1_TIM4EN)
    using Tim4Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_TIM4EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_USART5EN)
    using Usart5Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_USART5EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_USART6EN)
    using Usart6Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_USART6EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_USBEN)
    using UsbClock = ClockControl<PeriphClockEnable1, RCC_APBENR1_USBEN, ApbClock>;
#endif
#if defined (RCC_APBENR1_SPI3EN)
    using Spi3Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_SPI3EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_I2C3EN)
    using I2c3Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_I2C3EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_LPUART2EN)
    using LpUart2Clock = ClockControl<PeriphClockEnable1, RCC_APBENR1_LPUART2EN, ApbClock>;
#endif
#if defined (RCC_APBENR1_FDCANEN)
    using FdCanClock = ClockControl<PeriphClockEnable1, RCC_APBENR1_FDCANEN, ApbClock>;
#endif
#if defined (RCC_APBENR1_CRSEN)
    using CrsClock = ClockControl<PeriphClockEnable1, RCC_APBENR1_CRSEN, ApbClock>;
#endif

/* Sleep/Stop mode not implemented
    using GPIOASMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIOASMEN, ApbClock>;
    using GPIOBSMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIOBSMEN, ApbClock>;
    using GPIOCSMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIOCSMEN, ApbClock>;
    using GPIODSMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIODSMEN, ApbClock>;
    using GPIOFSMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIOFSMEN, ApbClock>;
    using DMA1SMClock = ClockControl<PeriphClockEnable, RCC_AHBSMENR_DMA1SMEN, ApbClock>;
    using FLASHSMClock = ClockControl<PeriphClockEnable, RCC_AHBSMENR_FLASHSMEN, ApbClock>;
    using SRAMSMClock = ClockControl<PeriphClockEnable, RCC_AHBSMENR_SRAMSMEN, ApbClock>;
    using CRCSMClock = ClockControl<PeriphClockEnable, RCC_AHBSMENR_CRCSMEN, ApbClock>;
    using TIM3SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_TIM3SMEN, ApbClock>;
    using RTCAPBSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_RTCAPBSMEN, ApbClock>;
    using WWDGSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_WWDGSMEN, ApbClock>;
    using SPI2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_SPI2SMEN, ApbClock>;
    using USART2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USART2SMEN, ApbClock>;
    using I2C1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_I2C1SMEN, ApbClock>;
    using I2C2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_I2C2SMEN, ApbClock>;
    using DBGSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_DBGSMEN, ApbClock>;
    using PWRSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_PWRSMEN, ApbClock>;
    using SYSCFGSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_SYSCFGSMEN, ApbClock>;
    using TIM1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_TIM1SMEN, ApbClock>;
    using SPI1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_SPI1SMEN, ApbClock>;
    using USART1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_USART1SMEN, ApbClock>;
    using TIM14SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_TIM14SMEN, ApbClock>;
    using TIM16SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_TIM16SMEN, ApbClock>;
    using TIM17SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_TIM17SMEN, ApbClock>;
    using ADCSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_ADCSMEN, ApbClock>;

#if defined (RCC_AHBSMENR_AESSMEN)
    using AESSMClock = ClockControl<AhbClockEnableReg, RCC_AHBSMENR_AESSMEN, AhbClock>;
#endif
#if defined (RCC_AHBSMENR_RNGSMEN)
    using RNGSMClock = ClockControl<AhbClockEnableReg, RCC_AHBSMENR_RNGSMEN, AhbClock>;
#endif

#if defined (RCC_APBSMENR1_TIM2SMEN)
    using TIM2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_TIM2SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_LPUART1SMEN)
    using LPUART1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_LPUART1SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_LPTIM2SMEN)
    using LPTIM2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_LPTIM2SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_LPTIM1SMEN)
    using LPTIM1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_LPTIM1SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_TIM6SMEN)
    using Tim6SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_TIM6SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_TIM7SMEN)
    using Tim7SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_TIM7SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR2_TIM15SMEN)
    using TIM15SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR2_TIM15SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_DAC1SMEN)
    using DAC1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_DAC1SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_USART3SMEN)
    using USART3SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USART3SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_USART4SMEN)
    using USART4SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USART4SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_CECSMEN)
    using CECSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_CECSMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_UCPD1SMEN)
    using UCPD1SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_UCPD1SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_UCPD2SMEN)
    using UCPD2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_UCPD2SMEN, ApbClock>;
#endif
#if defined (RCC_IOPSMENR_GPIOESMEN)
    using GPIOESMClock = ClockControl<PeriphClockEnable, RCC_IOPSMENR_GPIOESMEN, ApbClock>;
#endif
#if defined (RCC_AHBSMENR_DMA2SMEN)
    using DMA2SMClock = ClockControl<PeriphClockEnable, RCC_AHBSMENR_DMA2SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_TIM4SMEN)
    using TIM4SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_TIM4SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_USART5SMEN)
    using USART5SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USART5SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_USART6SMEN)
    using USART6SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USART6SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_USBSMEN)
    using USBSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_USBSMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_SPI3SMEN)
    using SPI3SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_SPI3SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_I2C3SMEN)
    using I2C3SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_I2C3SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_LPUART2SMEN)
    using LpUart2SMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_LPUART2SMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_FDCANSMEN)
    using FDCANSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_FDCANSMEN, ApbClock>;
#endif
#if defined (RCC_APBSMENR1_CRSSMEN)
    using CRSSMClock = ClockControl<PeriphClockEnable, RCC_APBSMENR1_CRSSMEN, ApbClock>;
#endif
*/
} // namespace Zhele::Clock

#endif //! ZHELE_CLOCK_H