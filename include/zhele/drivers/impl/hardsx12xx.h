/**
 * @file
 * Implements method of scdard class
 * 
 * @author Konstantin Chizhov
 * @date 2012
 * @license FreeBSD
 */

#ifndef ZHELE_DRIVERS_SXHARD_IMPL_H
#define ZHELE_DRIVERS_SXHARD_IMPL_H

#include  <zhele/drivers/hardsx12xx.h>

namespace Zhele::Drivers
{
    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    uint16_t SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::SpiCommand(uint8_t index, uint32_t arg, uint8_t crc)
    {
        _CsPin::Clear();
        //Spi.Read();
        Spi.Write(index | (1 << 6));
        Spi.WriteU32Be(arg);
        Spi.Write(crc | 1);
        uint16_t responce = Spi.IgnoreWhile(1000, 0xff);
//        if(index == SendStatus && responce !=0xff)
//            responce |= Spi.Read() << 8;
        _CsPin::Set();
        //Spi.Read();
        return responce;
    }


    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    void SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::write(uint8_t* _buff, uint8_t _numbytes)
    {
        _CsPin::Clear();
        Spi.Read(_buff, _numbytes);
        _CsPin::Set();
    }


    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    uint32_t SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::ReadBlocksCount()
    {
        uint8_t csd[16];
//        if(!SpiCommand(SendCsd, 0) && ReadDataBlock(csd, 16))
//        {
//            if(csd[0] & 0xC0) // SD v2
//            {
//                uint32_t c_size = (((uint32_t)csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
//                return (c_size + 1) * 1024u - 1u;
//            }else // SD v1
//            {
//                uint32_t c_size = ((((uint32_t)csd[6] << 16) | ((uint32_t)csd[7] << 8) | csd[8]) & 0x0003FFC0) >> 6;
//                uint16_t c_size_mult = ((uint16_t)((csd[9] & 0x03) << 1)) | ((uint16_t)((csd[10] & 0x80) >> 7));
//                uint16_t block_len = csd[5] & 0x0F;
//                block_len = 1u << (block_len - 9);
//                return ((c_size + 1u) * (1u << (c_size_mult + 2u)) * block_len) - 1u;
//            }
//        }
        return 0;
    }

    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    void SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::digitalWrite(uint8_t _pin, uint8_t _value)
    {
        // TODO: cache this value

        if(_pin == 0) {
            if(_value == 0) _AmpPin::Clear();
            else _AmpPin::Set();
        } else if(_pin == 1) {
            if(_value == 0) _TxPin::Clear();
            else _TxPin::Set();
        } else if(_pin == 2) {
            if(_value == 0) _RxPin::Clear();
            else _RxPin::Set();
        }
    }

    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    void SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::read(uint8_t* _buff, uint8_t _numbytes)
    {
        _CsPin::Clear();
         Spi.Read(_buff, _numbytes);
        _CsPin::Set();
    }

    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    bool SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::WaitWhileBusy()
    {
        _CsPin::Clear();
        return Spi.Ignore(10000u, 0xff) == 0xff;
    }
}

#endif //! ZHELE_DRIVERS_SDCARD_IMPL_H
