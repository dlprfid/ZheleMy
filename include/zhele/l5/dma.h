/**
 * @file
 * Implement DMA protocol for stm32f4 series
 * 
 * @author Alexey Zhelonkin
 * @date 2019
 * @license FreeBSD
 */

#ifndef ZHELE_DMA_H
#define ZHELE_DMA_H

#include <stm32l5xx.h>
#include "../common/dma.h"

namespace Zhele
{
    namespace Private
    {
        IO_STRUCT_WRAPPER(DMA1, Dma1, DMA_TypeDef);
        IO_STRUCT_WRAPPER(DMA2, Dma2, DMA_TypeDef);
        
        IO_STRUCT_WRAPPER(DMA1_Channel1, Dma1Stream1, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel2, Dma1Stream2, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel3, Dma1Stream3, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel4, Dma1Stream4, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel5, Dma1Stream5, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel6, Dma1Stream6, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA1_Channel7, Dma1Stream7, DMA_Channel_TypeDef);
        
        IO_STRUCT_WRAPPER(DMA2_Channel1, Dma2Stream1, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel2, Dma2Stream2, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel3, Dma2Stream3, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel4, Dma2Stream4, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel5, Dma2Stream5, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel6, Dma2Stream6, DMA_Channel_TypeDef);
        IO_STRUCT_WRAPPER(DMA2_Channel7, Dma2Stream7, DMA_Channel_TypeDef);

        template<typename _DmaStream, unsigned _DmaChannel>
        class DmaStreamChannel : public _DmaStream
        {
        public:
            static void Transfer(DmaBase::Mode mode, const void* buffer, volatile void* periph, uint32_t bufferSize)
            {
                _DmaStream::Transfer(mode, buffer, periph, bufferSize, _DmaChannel);
            }
        };
    }        

    // Alias for stream
    template<typename _Module, typename _StreamRegs, unsigned _Stream, IRQn_Type _IRQNumber>
    using DmaStream = DmaChannel<_Module, _StreamRegs, _Stream, _IRQNumber>;

    using Dma1 = DmaModule<Private::Dma1, Clock::Dma1Clock, 8>;
    using Dma1Stream1 = DmaStream<Dma1, Private::Dma1Stream1, 1, DMA1_Channel1_IRQn>;
    using Dma1Stream2 = DmaStream<Dma1, Private::Dma1Stream2, 2, DMA1_Channel2_IRQn>;
    using Dma1Stream3 = DmaStream<Dma1, Private::Dma1Stream3, 3, DMA1_Channel3_IRQn>;
    using Dma1Stream4 = DmaStream<Dma1, Private::Dma1Stream4, 4, DMA1_Channel4_IRQn>;
    using Dma1Stream5 = DmaStream<Dma1, Private::Dma1Stream5, 5, DMA1_Channel5_IRQn>;
    using Dma1Stream6 = DmaStream<Dma1, Private::Dma1Stream6, 6, DMA1_Channel6_IRQn>;
    using Dma1Stream7 = DmaStream<Dma1, Private::Dma1Stream7, 7, DMA1_Channel7_IRQn>;
    

    using Dma2 = DmaModule<Private::Dma2, Clock::Dma2Clock, 8>;
    using Dma2Stream1 = DmaStream<Dma2, Private::Dma2Stream1, 1, DMA2_Channel1_IRQn>;
    using Dma2Stream2 = DmaStream<Dma2, Private::Dma2Stream2, 2, DMA2_Channel2_IRQn>;
    using Dma2Stream3 = DmaStream<Dma2, Private::Dma2Stream3, 3, DMA2_Channel3_IRQn>;
    using Dma2Stream4 = DmaStream<Dma2, Private::Dma2Stream4, 4, DMA2_Channel4_IRQn>;
    using Dma2Stream5 = DmaStream<Dma2, Private::Dma2Stream5, 5, DMA2_Channel5_IRQn>;
    using Dma2Stream6 = DmaStream<Dma2, Private::Dma2Stream6, 6, DMA2_Channel6_IRQn>;
    using Dma2Stream7 = DmaStream<Dma2, Private::Dma2Stream7, 7, DMA2_Channel7_IRQn>;

#define DECLARE_STREAM_CHANNELS(DMA_MODULE_NUMBER, DMA_STREAM_NUMBER) \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel0 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 0>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel1 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 1>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel2 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 2>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel3 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 3>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel4 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 4>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel5 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 5>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel6 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 6>; \
    using Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER##Channel7 = Private::DmaStreamChannel<Dma##DMA_MODULE_NUMBER##Stream##DMA_STREAM_NUMBER, 7>; \

    DECLARE_STREAM_CHANNELS(1, 1)
    DECLARE_STREAM_CHANNELS(1, 2)
    DECLARE_STREAM_CHANNELS(1, 3)
    DECLARE_STREAM_CHANNELS(1, 4)
    DECLARE_STREAM_CHANNELS(1, 5)
    DECLARE_STREAM_CHANNELS(1, 6)
    DECLARE_STREAM_CHANNELS(1, 7)

    DECLARE_STREAM_CHANNELS(2, 1)
    DECLARE_STREAM_CHANNELS(2, 2)
    DECLARE_STREAM_CHANNELS(2, 3)
    DECLARE_STREAM_CHANNELS(2, 4)
    DECLARE_STREAM_CHANNELS(2, 5)
    DECLARE_STREAM_CHANNELS(2, 6)
    DECLARE_STREAM_CHANNELS(2, 7)
}

#endif //! ZHELE_DMA_H