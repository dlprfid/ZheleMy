/**
 * @file
 * Driver for SD card
 * 
 * @author Konstantin Chizhov
 * @date 2012
 * @license FreeBSD
 */

#ifndef ZHELE_DRIVERS_SXHARD_H
#define ZHELE_DRIVERS_SXHARD_H

#include <stdlib.h>
#include <zhele/delay.h>
#include <zhele/binary_stream.h>

namespace Zhele::Drivers
{

    /**
     * @brief Implements SD card
     * 
     * @tparam _SpiModule SPI module
     * @tparam _CsPin Chip select pin
     */
    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    class SxHard
    {
        using SpiModule = _SpiModule;
        using CsPin = _CsPin;
        using AmpPin = _AmpPin;
        using TxPin = _TxPin;
        using RstPin = _RstPin;
        using RxPin = _RxPin;
        using PortExti = _PortExti;
        using LineExti = _LineExti;




        static const uint16_t CommandTimeoutValue = 100;            ///< Command timeout
        static const bool useCrc = false;                           ///< CRC using flag
//        static SdCardType _type;                                    ///< SD card type
        static BinaryStream<_SpiModule> Spi;                        ///< Binary stream
    
    protected:
        /**
         * @brief Execute spi command
         * 
         * @param index Index
         * @param arg Argument
         * @param crc Crc (default 0)
         * @return uint16_t Command result
         */
        static uint16_t SpiCommand(uint8_t index, uint32_t arg, uint8_t crc = 0);

        /**
         * @brief Read card blocks count
         * 
         * @return uint32_t Blocks count
         */
        static uint32_t ReadBlocksCount();

        /**
         * @brief Wait while bus is busy
         * 
         * @return true Bus free
         * @return false Timeout
         */
        static bool WaitWhileBusy();

        /**
         * @brief Read data block
         * 
         * @tparam ReadIterator Iterator type
         * 
         * @param iter Iterator
         * @param size Size to read
         * @return true Read success
         * @return false Read failed
         */
        template<typename ReadIterator>
        static bool ReadDataBlock(ReadIterator iter, size_t size)
        {
            _CsPin::Clear();

            uint8_t resp;
            resp = Spi.IgnoreWhile(1000, 0xFF);
            if(resp != 0xFE)
            {
                _CsPin::Set();
                return false;
            }
            Spi. template Read<ReadIterator>(iter, size);
            uint16_t crc = Spi.ReadU16Le();
            if(useCrc)
            {
                // TBD
            }
            else
            {
                (void)crc;
            }
            _CsPin::Set();
            Spi.Read();
            return true;
        }

    public:
        /**
         * @brief Check card status
         * 
         * @return true OK
         * @return false Error
         */
        static void digitalWrite(uint8_t _pin, uint8_t _value);

        /**
         * @brief Returns card's blocks count
         * 
         * @return uint32_t Blocks count
         */
        static void read(uint8_t* _buff, uint8_t _numbytes);

        /**
         * @brief Returns card's block size
         * 
         * @return size_t Block size
         */
        static void write(uint8_t* _buff, uint8_t _numbytes);

//        /**
//         * @brief Writes block to card
//         *
//         * @tparam WriteIterator Iterator type
//         *
//         * @param iter Iterator
//         * @param logicalBlockAddress Block address
//         *
//         * @return true Success
//         * @return false Fail
//         */
//        template<typename WriteIterator>
//        static bool WriteBlock(WriteIterator iter, uint32_t logicalBlockAddress)
//        {
//            if(_type != SdhcCard)
//                logicalBlockAddress <<= 9;
//            if(SpiCommand(SdCardCommand::WriteBlock, logicalBlockAddress) == 0)
//            {
//                _CsPin::Clear();
//                if(Spi.Ignore(10000u, 0xff) != 0xff)
//                {
//                    return false;
//                }
//
//                Spi.Write(0xFE);
//                Spi.template Write<WriteIterator>(iter, 512);
//                Spi.ReadU16Be();
//                uint8_t resp;
//                if((resp = Spi.Read() & 0x1F) != 0x05)
//                {
//                    return false;
//                }
//                _CsPin::Set();
//                Spi.Read();
//                return true;
//            }
//            return false;
//        }

        /**
         * @brief Read block from card
         * 
         * @tparam ReadIterator Iterator type
         * 
         * @param iter Iterator
         * @param logicalBlockAddress Block address
         * 
         * @return true Success
         * @return false Fail
         */
//        template<typename ReadIterator>
//        static bool ReadBlock(ReadIterator iter, uint32_t logicalBlockAddress)
//        {
//            if(_type != SdhcCard)
//                logicalBlockAddress <<= 9;
//            if(!WaitWhileBusy())
//                return false;
//            if(SpiCommand(ReadSingleBlock, logicalBlockAddress) == 0)
//            {
//                return ReadDataBlock<ReadIterator>(iter, 512);
//            }
//            return false;
//        }

        /**
         * @brief Read multiple blocks from card
         * 
         * @tparam ReadIterator Iterator type
         * 
         * @param iter [in] Iterator
         * @param logicalBlockAddress [in] Block address
         * @param [in] blocksCount Block to read count
         * 
         * @return true Success
         * @return false Fail
         */
//        template<typename ReadIterator>
//        static bool ReadMultipleBlock(ReadIterator iter, uint32_t logicalBlockAddress, uint32_t blocksCount)
//        {
//            if(_type != SdhcCard)
//                logicalBlockAddress <<= 9;
//            if(!WaitWhileBusy())
//                return false;
//            if(SpiCommand(SdCardCommand::ReadMultipleBlock, logicalBlockAddress) == 0)
//            {
//                ReadDataBlock<ReadIterator>(iter, 512 * blocksCount);
//            }
//            return false;
//        }
    };

    template<typename _SpiModule, typename _CsPin,typename _AmpPin, typename _TxPin, typename _RxPin, typename _PortExti, typename _LineExti, typename _RstPin>
    BinaryStream<_SpiModule> SxHard<_SpiModule, _CsPin, _AmpPin, _TxPin, _RxPin, _PortExti, _LineExti, _RstPin>::Spi;

} // namespace Zhele::Drivers

#include "impl/sdcard.h"


#endif //! ZHELE_DRIVERS_SDCARD_H
