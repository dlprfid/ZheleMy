/**
 * @file
 * Implement USB device class methods
 * 
 * @author Aleksei Zhelonkin
 * @date 2021
 * @license FreeBSD
 */

#ifndef ZHELE_USB_DEVICE_IMPL_H
#define ZHELE_USB_DEVICE_IMPL_H

namespace Zhele::Usb
{
#if defined (USB)
    #define USB_DEVICE_TEMPLATE_ARGS template< \
        typename _Regs, \
        IRQn_Type _IRQNumber, \
        typename _ClockCtrl, \
        uint16_t _UsbVersion, \
        DeviceAndInterfaceClass _Class, \
        uint8_t _SubClass, \
        uint8_t _Protocol, \
        uint16_t _VendorId, \
        uint16_t _ProductId, \
        uint16_t _DeviceReleaseNumber, \
        auto _Manufacturer, \
        auto _Product, \
        auto _Serial, \
        typename _Ep0, \
        typename... _Configurations>
    #define USB_DEVICE_TEMPLATE_QUALIFIER DeviceBase<_Regs, _IRQNumber, _ClockCtrl, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, _Manufacturer, _Product, _Serial, _Ep0, _Configurations...>
#elif defined (USB_OTG_FS)
    #define USB_DEVICE_TEMPLATE_ARGS template< \
        typename _Regs, \
        typename _DeviceRegs, \
        IRQn_Type _IRQNumber, \
        typename _ClockCtrl, \
        uint16_t _UsbVersion, \
        DeviceAndInterfaceClass _Class, \
        uint8_t _SubClass, \
        uint8_t _Protocol, \
        uint16_t _VendorId, \
        uint16_t _ProductId, \
        uint16_t _DeviceReleaseNumber, \
        auto _Manufacturer, \
        auto _Product, \
        auto _Serial, \
        typename _Ep0, \
        typename... _Configurations>
    #define USB_DEVICE_TEMPLATE_QUALIFIER DeviceBase<_Regs, _DeviceRegs, _IRQNumber, _ClockCtrl, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, _Manufacturer, _Product, _Serial, _Ep0, _Configurations...>
#endif

#if defined (USB)
    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Enable()
    {
        _ClockCtrl::Enable();
        _epBufferManager.Init();

        _Regs()->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
        _Regs()->ISTR = 0;
        _Regs()->BTABLE = 0;
#if defined (USB_BCDR_DPPU)
        _Regs()->BCDR |= USB_BCDR_DPPU;
#endif
        NVIC_EnableIRQ(_IRQNumber);
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Reset()
    {
        _Ep0::Reset();
        
        (_Configurations::Reset(), ...);

        _Regs()->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
        _Regs()->ISTR = 0;
        _Regs()->BTABLE = 0;
        _Regs()->DADDR = USB_DADDR_EF;
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::CommonHandler()
    {
        NVIC_ClearPendingIRQ(_IRQNumber);

        if(_Regs()->ISTR & USB_ISTR_RESET)
        {
            Reset();
        }
        if (_Regs()->ISTR & USB_ISTR_CTR)
        {
            uint8_t endpoint = _Regs()->ISTR & USB_ISTR_EP_ID;
            _epHandlers.Handle(endpoint, ((_Regs()->ISTR & USB_ISTR_DIR) != 0 ? EndpointDirection::Out : EndpointDirection::In));
        }
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Handler()
    {
        if(_Ep0::Reg::Get() & USB_EP_CTR_RX)
        {
            _Ep0::ClearCtrRx();
            
            if(_Ep0::Reg::Get() & USB_EP_SETUP)
            {
                HandleSetupRequest(reinterpret_cast<SetupPacket*>(_Ep0::RxBuffer));
            }
            else
            {
                _Ep0::TryHandleDataTransfer();
            }
        }
        if(_Ep0::Reg::Get() & USB_EP_CTR_TX)
        {
            _Ep0::ClearCtrTx();
            _Ep0::HandleTx();
        }
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::SetAddress(uint16_t address)
    {
        _tempAddressStorage = address;
        _Ep0::SendZLP([](){
            _Regs()->DADDR = (USB_DADDR_EF | (_tempAddressStorage & USB_DADDR_ADD));
            _Ep0::SetRxStatus(EndpointStatus::Valid);
        });
    }
#elif defined (USB_OTG_FS)
    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Enable()
    {
        _ClockCtrl::Enable();
        
        _epBufferManager.Init();


        while (!(_Regs()->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)) continue;

        _DeviceRegs()->DCFG = USB_OTG_DCFG_DSPD;

        _Regs()->GUSBCFG = USB_OTG_GUSBCFG_FDMOD // Force device mode
                        | (0x06 << USB_OTG_GUSBCFG_TRDT_Pos) // ??
                        | USB_OTG_GUSBCFG_PHYSEL; // USB 2.0

        _Regs()->GCCFG = USB_OTG_GCCFG_NOVBUSSENS; // Exit Power Down mode
        _Regs()->GINTSTS = 0xfffffffful;

        _Regs()->GINTMSK = USB_OTG_GINTMSK_IEPINT   // Enable USB IN TX endpoint interrupt
                            | USB_OTG_GINTMSK_OEPINT   // Enable USB OUT RX endpoint interrupt
                            | USB_OTG_GINTMSK_RXFLVLM // USB reciving
                            | USB_OTG_GINTMSK_USBRST
                            | USB_OTG_GINTMSK_ENUMDNEM;    // Reset interrupt

        _Regs()->GCCFG |= USB_OTG_GCCFG_PWRDWN; // Enable USB
        _DeviceRegs()->DCTL = 0;

        NVIC_EnableIRQ(_IRQNumber);
        _Regs()->GAHBCFG = USB_OTG_GAHBCFG_GINT;       // On USB general interrupt
    }

    template<typename _Regs>
    void FlushTx()
    {
        while ((_Regs()->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) continue;

        _Regs()->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (1 << 10);
        while(_Regs()->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) continue;;
    }
        
    template<typename _Regs>
    void FlushRx()
    {
        while ((_Regs()->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) continue;
        _Regs()->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
        while (_Regs()->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);
    }  

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Reset()
    {
        for (auto i = 0; i < 3; ++i)
            _Regs()->DIEPTXF[i] = 0;

        _epBufferManager.Init();

        _Ep0::Reset();
        (_Configurations::Reset(), ...);

        _DeviceRegs()->DAINTMSK = CalculateDaintMask(_endpoints.template push_back<This>());
        _DeviceRegs()->DOEPMSK = USB_OTG_DOEPMSK_STUPM // Enable setup-done irq
                                | USB_OTG_DOEPMSK_XFRCM;  // Enable tx-done irq

        _DeviceRegs()->DIEPMSK = USB_OTG_DIEPMSK_XFRCM; // Enable transfer complete irq

        FlushTx<_Regs>();
        FlushRx<_Regs>();
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::CommonHandler()
    {
        if (_Regs()->GINTSTS & USB_OTG_GINTSTS_USBRST) {
            _Regs()->GINTSTS = USB_OTG_GINTSTS_USBRST;
            Reset();
        }

        if (_Regs()->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) { 
            _Regs()->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
            _isDeviceConfigured = true;
        } 

        if (_Regs()->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
            uint32_t status = _Regs()->GRXSTSP;
            uint16_t size = (status & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
            uint8_t enpointNumber = status & USB_OTG_GRXSTSP_EPNUM;

            _epFifoNotEmptyHandlers.HandleRxFifoNotEmpty(enpointNumber, size);
        }

        if(_Regs()->GINTSTS & USB_OTG_GINTSTS_OEPINT) {
            uint32_t endpoints = _DeviceRegs()->DAINT & _DeviceRegs()->DAINTMSK;

            if (endpoints & (1 << 16)) {
                _epHandlers.Handle(0, EndpointDirection::Out);
            }
            if (endpoints & (1 << 17)) {
                _epHandlers.Handle(1, EndpointDirection::Out);
            }
            if (endpoints & (1 << 18)) {
                _epHandlers.Handle(2, EndpointDirection::Out);
            }
            if (endpoints & (1 << 19)) {
                _epHandlers.Handle(3, EndpointDirection::Out);
            }
        }

        if(_Regs()->GINTSTS & (USB_OTG_GINTSTS_IEPINT | USB_OTG_GINTSTS_NPTXFE)) {
            uint32_t endpoints = _DeviceRegs()->DAINT & _DeviceRegs()->DAINTMSK;

            if (endpoints & (1 << 0)) {
                _epHandlers.Handle(0, EndpointDirection::In);
            }
            if (endpoints & (1 << 1)) {
                _epHandlers.Handle(1, EndpointDirection::In);
            }
            if (endpoints & (1 << 2)) {
                _epHandlers.Handle(2, EndpointDirection::In);
            }
            if (endpoints & (1 << 3)) {
                _epHandlers.Handle(3, EndpointDirection::In);
            }
        }
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::Handler()
    {
        if (_Ep0::GetOutInterrupts() & USB_OTG_DOEPINT_STUP) {
            HandleSetupRequest(reinterpret_cast<SetupPacket*>(_Ep0::RxBuffer));
#if defined(USB_OTG_FS)
        _Ep0::SetRxStatus(EndpointStatus::Valid);
#endif
        }
        if (_Ep0::GetOutInterrupts() & USB_OTG_DOEPINT_XFRC) {
            _Ep0::TryHandleDataTransfer();
        }

        _Ep0::ClearAllRxInterrupts();

        _Ep0::HandleTx();
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::SetAddress(uint16_t address)
    {        
        _DeviceRegs()->DCFG |= static_cast<uint32_t>(address) << USB_OTG_DCFG_DAD_Pos;
        _Ep0::SendZLP();
    }

#endif
    USB_DEVICE_TEMPLATE_ARGS
    bool USB_DEVICE_TEMPLATE_QUALIFIER::IsDeviceConfigured()
    {
        return _isDeviceConfigured;
    }

    USB_DEVICE_TEMPLATE_ARGS
    consteval DeviceDescriptor USB_DEVICE_TEMPLATE_QUALIFIER::BuildDeviceDescriptor()
    {
        return DeviceDescriptor {
            .UsbVersion = _UsbVersion,
            .Class = _Class,
            .SubClass = _SubClass,
            .Protocol = _Protocol,
            .MaxPacketSize = _Ep0::MaxPacketSize,
            .VendorId = _VendorId,
            .ProductId = _ProductId,
            .DeviceReleaseNumber = _DeviceReleaseNumber,
            .ManufacturerStringIndex = (std::is_same_v<decltype(_Manufacturer), decltype(EmptyFixedString16)>) ? 0 : 1,
            .ProductStringIndex = (std::is_same_v<decltype(_Product), decltype(EmptyFixedString16)>) ? 0 : 2,
            .SerialNumberStringIndex = (std::is_same_v<decltype(_Serial), decltype(EmptyFixedString16)>) ? 0 : 3,
            .ConfigurationsCount = sizeof...(_Configurations)
        };
    }

    USB_DEVICE_TEMPLATE_ARGS
    consteval unsigned USB_DEVICE_TEMPLATE_QUALIFIER::ConfigurationDescriptorSize()
    {
        uint16_t size = 0;

        _configurations.foreach([&size](auto configuration) {
            size += TypeUnbox<configuration>::GetDescriptor().size();
        });

        return size;
    }


    USB_DEVICE_TEMPLATE_ARGS
    consteval auto USB_DEVICE_TEMPLATE_QUALIFIER::BuildConfigurationDescriptor()
    {
        constexpr auto size = ConfigurationDescriptorSize();

        std::array<uint8_t, size> result;
        auto dst = result.begin();

        _configurations.foreach([&dst](auto configuration) {
            auto nextConfigurationDescriptor = TypeUnbox<configuration>::GetDescriptor();
            dst = std::copy(nextConfigurationDescriptor.begin(), nextConfigurationDescriptor.end(), dst);
        });

        return result;
    }

    USB_DEVICE_TEMPLATE_ARGS
    inline consteval auto USB_DEVICE_TEMPLATE_QUALIFIER::BuildStringDescriptor(auto str)
    {
        std::array<uint16_t, str.Size / 2 + 1> result {
            ((2 + str.Size) << 8) | static_cast<uint8_t>(DescriptorType::String)
        };
        auto dst = result.begin();
        ++dst;

        std::copy(str.Text, str.Text + str.Size / 2, dst);

        return result;
    }

    USB_DEVICE_TEMPLATE_ARGS
    void USB_DEVICE_TEMPLATE_QUALIFIER::HandleSetupRequest(SetupPacket* setupRequest)
    {
        if (setupRequest->RequestType.Recipient == 1) {
            _ifHandlers.HandleSetupRequest(setupRequest->Index & 0xff);
            return;
        }
        
        switch (setupRequest->Request) {
        case StandartRequestCode::GetStatus: {
            uint16_t status = 0;
            _Ep0::SendData(&status, sizeof(status));
            break;
        }
        case StandartRequestCode::SetAddress: {
            SetAddress(setupRequest->Value);
            break;
        }

        case StandartRequestCode::GetDescriptor: {
            switch (static_cast<GetDescriptorParameter>(setupRequest->Value)) {
            case GetDescriptorParameter::DeviceDescriptor: {
                constexpr auto descriptor = BuildDeviceDescriptor();
                _Ep0::SendData(&descriptor, setupRequest->Length < sizeof(descriptor) ? setupRequest->Length : sizeof(descriptor));
                break;
            }

            case GetDescriptorParameter::ConfigurationDescriptor: {
                constexpr auto descriptor = BuildConfigurationDescriptor();
                _Ep0::SendData(descriptor.data(), setupRequest->Length < descriptor.size() ? setupRequest->Length : descriptor.size());
                break;
            }
            case GetDescriptorParameter::StringLangDescriptor: {
                LangIdDescriptor langIdDescriptor;
                _Ep0::SendData(&langIdDescriptor, setupRequest->Length < sizeof(langIdDescriptor) ? setupRequest->Length : sizeof(langIdDescriptor));
                break;
            }
            case GetDescriptorParameter::StringManDescriptor: {
                if constexpr (!std::is_same_v<decltype(_Manufacturer), decltype(EmptyFixedString16)>)
                {
                    constexpr auto descriptor = BuildStringDescriptor(_Manufacturer);
                    constexpr auto size = descriptor.size() * sizeof(descriptor[0]);
                    _Ep0::SendData(descriptor.data(), setupRequest->Length < size ? setupRequest->Length : size);
                    break;
                }
            }

            case GetDescriptorParameter::StringProdDescriptor: {
                if constexpr (!std::is_same_v<decltype(_Product), decltype(EmptyFixedString16)>)
                {
                    constexpr auto descriptor = BuildStringDescriptor(_Product);
                    constexpr auto size = descriptor.size() * sizeof(descriptor[0]);
                    _Ep0::SendData(descriptor.data(), setupRequest->Length < size ? setupRequest->Length : size);
                    break;
                }
            }
            case GetDescriptorParameter::StringSerialNumberDescriptor: {
                if constexpr (!std::is_same_v<decltype(_Serial), decltype(EmptyFixedString16)>)
                {
                    constexpr auto descriptor = BuildStringDescriptor(_Serial);
                    constexpr auto size = descriptor.size() * sizeof(descriptor[0]);
                    _Ep0::SendData(descriptor.data(), setupRequest->Length < size ? setupRequest->Length : size);
                    break;
                }
            }
            default:
                _Ep0::SetTxStatus(EndpointStatus::Stall);
                break;
            }
            break;
        }
        case StandartRequestCode::GetConfiguration: {
            uint8_t response = _isDeviceConfigured ? 1 : 0;
            _Ep0::SendData(&response, 1);
            break;
        }
        case StandartRequestCode::SetConfiguration: {
            _isDeviceConfigured = true;
            _Ep0::SendZLP();
            break;
        }
        default:
            _Ep0::SetTxStatus(EndpointStatus::Stall);
            break;
        }
    }

#if defined (USB)
    IO_STRUCT_WRAPPER(USB, UsbRegs, USB_TypeDef);
#elif defined (USB_OTG_FS)
    IO_STRUCT_WRAPPER(USB_OTG_FS, UsbRegs, USB_OTG_GlobalTypeDef);
    IO_STRUCT_WRAPPER(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE, UsbDeviceRegs, USB_OTG_DeviceTypeDef);
#endif

#if defined (USB_LP_IRQn)
    #define USB_IRQ USB_LP_IRQn
#elif defined (USB)
    #define USB_IRQ USB_IRQn
#elif defined (USB_OTG_FS)
    #define USB_IRQ OTG_FS_IRQn
#endif

#if defined (USB)
    using UsbClock = Zhele::Clock::UsbClock;
#elif defined (USB_OTG_FS)
    using UsbClock = Zhele::Clock::OtgFsClock;
#endif
    template<
        uint16_t _UsbVersion,
        DeviceAndInterfaceClass _Class,
        uint8_t _SubClass,
        uint8_t _Protocol,
        uint16_t _VendorId,
        uint16_t _ProductId,
        uint16_t _DeviceReleaseNumber,
        auto _Manufacturer,
        auto _Product,
        auto _Serial,
        typename _Ep0,
        typename... _Configurations>
#if defined (USB)
    using DeviceWithStrings = DeviceBase<UsbRegs, USB_IRQ, UsbClock, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, _Manufacturer, _Product, _Serial, _Ep0, _Configurations...>;
#elif defined (USB_OTG_FS)
    using DeviceWithStrings = DeviceBase<UsbRegs, UsbDeviceRegs, USB_IRQ, UsbClock, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, _Manufacturer, _Product, _Serial, _Ep0, _Configurations...>;
#endif
    template<
        uint16_t _UsbVersion,
        DeviceAndInterfaceClass _Class,
        uint8_t _SubClass,
        uint8_t _Protocol,
        uint16_t _VendorId,
        uint16_t _ProductId,
        uint16_t _DeviceReleaseNumber,
        typename _Ep0,
        typename... _Configurations>
#if defined (USB)
    using Device = DeviceBase<UsbRegs, USB_IRQ, UsbClock, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, EmptyFixedString16, EmptyFixedString16, EmptyFixedString16, _Ep0, _Configurations...>;
#elif defined (USB_OTG_FS)
    using Device = DeviceBase<UsbRegs, UsbDeviceRegs, USB_IRQ, UsbClock, _UsbVersion, _Class, _SubClass, _Protocol, _VendorId, _ProductId, _DeviceReleaseNumber, EmptyFixedString16, EmptyFixedString16, EmptyFixedString16, _Ep0, _Configurations...>;
#endif
#if defined (USB)
    USB_DEVICE_TEMPLATE_ARGS
    uint8_t USB_DEVICE_TEMPLATE_QUALIFIER::_tempAddressStorage = 0x00;
#endif
    USB_DEVICE_TEMPLATE_ARGS
    volatile bool USB_DEVICE_TEMPLATE_QUALIFIER::_isDeviceConfigured = false;
}

#endif //! ZHELE_USB_DEVICE_IMPL_H