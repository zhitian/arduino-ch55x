#include "USBhandler.h"

#include "USBconstant.h"

//CDC functions:
void resetCDCParameters();
void setLineCodingHandler();
uint16_t getLineCodingHandler();
void setControlLineStateHandler();
void USB_EP2_IN();
void USB_EP2_OUT();

__xdata __at (EP0_ADDR) uint8_t  Ep0Buffer[8];     
__xdata __at (EP1_ADDR) uint8_t  Ep1Buffer[8];       //on page 47 of data sheet, the receive buffer need to be min(possible packet size+2,64)
__xdata __at (EP2_ADDR) uint8_t  Ep2Buffer[128];     //IN and OUT buffer, must be even address

uint16_t SetupLen;
uint8_t SetupReq,UsbConfig;

__code uint8_t *pDescr;

volatile uint8_t usbMsgFlags=0;    // uint8_t usbMsgFlags copied from VUSB

inline void NOP_Process(void) {}

void USB_EP0_SETUP(){
    uint8_t len = USB_RX_LEN;
    if(len == (sizeof(USB_SETUP_REQ)))
    {
        SetupLen = ((uint16_t)UsbSetupBuf->wLengthH<<8) | (UsbSetupBuf->wLengthL);
        len = 0;                                                      // 默认为成功并且上传0长度
        SetupReq = UsbSetupBuf->bRequest;
        usbMsgFlags = 0;
        if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )//非标准请求
        {
            
            //here is the commnunication starts, refer to usbFunctionSetup of USBtiny
            //or usb_setup in usbtiny
            
            switch( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ))
            {
                case USB_REQ_TYP_VENDOR:    
                {
                    switch( SetupReq )
                    {
                        default:
                            len = 0xFF;                                                                        //command not supported
                            break;
                    }
                    break;
                }
                case USB_REQ_TYP_CLASS:
                {
                    switch( SetupReq )
                    {
                        case GET_LINE_CODING:   //0x21  currently configured
                            len = getLineCodingHandler();
                            break;
                        case SET_CONTROL_LINE_STATE:  //0x22  generates RS-232/V.24 style control signals
                            setControlLineStateHandler();
                            break;
                        case SET_LINE_CODING:      //0x20  Configure
                            break;
                            
                        default:
                            len = 0xFF;                                                                        /*命令不支持*/
                            break;
                    }
                    break;
                }
                default:
                    len = 0xFF;                                                                        /*命令不支持*/
                    break;
            }

        }
        else                                                             //标准请求
        {
            switch(SetupReq)                                             //请求码
            {
                case USB_GET_DESCRIPTOR:
                    switch(UsbSetupBuf->wValueH)
                {
                    case 1:                                                       //设备描述符
                        pDescr = DevDesc;                                         //把设备描述符送到要发送的缓冲区
                        len = DevDescLen;
                        break;
                    case 2:                                                        //配置描述符
                        pDescr = CfgDesc;                                          //把设备描述符送到要发送的缓冲区
                        len = CfgDescLen;
                        break;
                    case 3:
                        if(UsbSetupBuf->wValueL == 0)
                        {
                            pDescr = LangDes;
                            len = LangDesLen;
                        }
                        else if(UsbSetupBuf->wValueL == 1)
                        {
                            pDescr = Manuf_Des;
                            len = Manuf_DesLen;
                        }
                        else if(UsbSetupBuf->wValueL == 2)
                        {
                            pDescr = Prod_Des;
                            len = Prod_DesLen;
                        }
                        else if(UsbSetupBuf->wValueL == 3)
                        {
                            pDescr = SerDes;
                            len = SerDesLen;
                        }
                        else if(UsbSetupBuf->wValueL == 4)
                        {
                            pDescr = CDC_Des;
                            len = CDC_DesLen;
                        }
                        else
                        {
                            pDescr = SerDes;
                            len = SerDesLen;
                        }
                        break;
                    default:
                        len = 0xff;                                                //不支持的命令或者出错
                        break;
                }
                    if (len != 0xff){
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                            //本次传输长度
                        for (uint8_t i=0;i<len;i++){
                            Ep0Buffer[i] = pDescr[i];
                        }
                        //memcpy(Ep0Buffer,pDescr,len);                                  //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                    }
                    break;
                case USB_SET_ADDRESS:
                    SetupLen = UsbSetupBuf->wValueL;                              //暂存USB设备地址
                    break;
                case USB_GET_CONFIGURATION:
                    Ep0Buffer[0] = UsbConfig;
                    if ( SetupLen >= 1 )
                    {
                        len = 1;
                    }
                    break;
                case USB_SET_CONFIGURATION:
                    UsbConfig = UsbSetupBuf->wValueL;
                    break;
                case USB_GET_INTERFACE:
                    break;
                case USB_SET_INTERFACE:
                    if (UsbSetupBuf->wIndexL == 3){ //setting interface 3 (webusb)
                        //serial.js do selectAlternateInterface first
                    }
                    break;
                case USB_CLEAR_FEATURE:                                            //Clear Feature
                    if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )                  /* 清除设备 */
                    {
                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                        {
                            if( CfgDesc[ 7 ] & 0x20 )
                            {
                                /* 唤醒 */
                            }
                            else
                            {
                                len = 0xFF;                                        /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF;                                            /* 操作失败 */
                        }
                    }
                    else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                    {
                        switch( UsbSetupBuf->wIndexL )
                        {
                            case 0x84:
                                UEP4_CTRL = UEP4_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x04:
                                UEP4_CTRL = UEP4_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x03:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                         // 不支持的端点
                                break;
                        }
                    }
                    else
                    {
                        len = 0xFF;                                                // 不是端点不支持
                    }
                    break;
                case USB_SET_FEATURE:                                          /* Set Feature */
                    if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )                  /* 设置设备 */
                    {
                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                        {
                            if( CfgDesc[ 7 ] & 0x20 )
                            {
                                /* 休眠 */
#ifdef DE_PRINTF
                                sendStrDebugCodeSpace( "suspend\r\n" );                                                             //睡眠状态
#endif
                                //while ( XBUS_AUX & bUART0_TX );    //等待发送完成
                                //SAFE_MOD = 0x55;
                                //SAFE_MOD = 0xAA;
                                //WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;                      //USB或者RXD0/1有信号时可被唤醒
                                //PCON |= PD;                                                                 //睡眠
                                //SAFE_MOD = 0x55;
                                //SAFE_MOD = 0xAA;
                                //WAKE_CTRL = 0x00;
                            }
                            else
                            {
                                len = 0xFF;                                        /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF;                                            /* 操作失败 */
                        }
                    }
                    else if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP )             /* 设置端点 */
                    {
                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                        {
                            switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                            {
                                case 0x84:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点4 IN STALL */
                                    break;
                                case 0x04:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点4 OUT Stall */
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
                                    break;
                                case 0x03:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                case 0x01:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点1 OUT Stall */
                                default:
                                    len = 0xFF;                                    /* 操作失败 */
                                    break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      /* 操作失败 */
                        }
                    }
                    else
                    {
                        len = 0xFF;                                          /* 操作失败 */
                    }
                    break;
                case USB_GET_STATUS:
                    Ep0Buffer[0] = 0x00;
                    Ep0Buffer[1] = 0x00;
                    if ( SetupLen >= 2 )
                    {
                        len = 2;
                    }
                    else
                    {
                        len = SetupLen;
                    }
                    break;
                default:
                    len = 0xff;                                                    //操作失败
                    break;
            }
        }
    }
    else
    {
        len = 0xff;                                                         //包长度错误
    }
    if(len == 0xff)
    {
        SetupReq = 0xFF;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
    }
    else if(len <= DEFAULT_ENDP0_SIZE)                                                       //上传数据或者状态阶段返回0长度包
    {
        UEP0_T_LEN = len;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
    }
    else
    {
        UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
    }
}

void USB_EP0_IN(){
    switch(SetupReq)
    {
        case USB_GET_DESCRIPTOR:
        {
            uint8_t len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                 //本次传输长度
            for (uint8_t i=0;i<len;i++){
                Ep0Buffer[i] = pDescr[i];
            }
            //memcpy( Ep0Buffer, pDescr, len );                                   //加载上传数据
            SetupLen -= len;
            pDescr += len;
            UEP0_T_LEN = len;
            UEP0_CTRL ^= bUEP_T_TOG;                    //同步标志位翻转
        }
            break;
        case USB_SET_ADDRESS:
            USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
        default:
            UEP0_T_LEN = 0;                                                      //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
    }
}

void USB_EP0_OUT(){
    if(SetupReq ==SET_LINE_CODING)  //设置串口属性
    {
        if( U_TOG_OK )
        {
            setLineCodingHandler();
            UEP0_T_LEN = 0;
            UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // 准备上传0包
        }
    }
    else
    {
        UEP0_T_LEN = 0;
        UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //状态阶段，对IN响应NAK
    }
}


void USB_EP1_IN(){
    UEP1_T_LEN = 0;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
}


#pragma save
#pragma nooverlay
void USBInterrupt(void) {   //inline not really working in multiple files in SDCC
    if(UIF_TRANSFER) {
        // Dispatch to service functions
        uint8_t callIndex=USB_INT_ST & MASK_UIS_ENDP;
        switch (USB_INT_ST & MASK_UIS_TOKEN) {
            case UIS_TOKEN_OUT:
            {//SDCC will take IRAM if array of function pointer is used.
                switch (callIndex) {
                    case 0: EP0_OUT_Callback(); break;
                    case 1: EP1_OUT_Callback(); break;
                    case 2: EP2_OUT_Callback(); break;
                    case 3: EP3_OUT_Callback(); break;
                    case 4: EP4_OUT_Callback(); break;
                    default: break;
                }
            }
                break;
            case UIS_TOKEN_SOF:
            {//SDCC will take IRAM if array of function pointer is used.
                switch (callIndex) {
                    case 0: EP0_SOF_Callback(); break;
                    case 1: EP1_SOF_Callback(); break;
                    case 2: EP2_SOF_Callback(); break;
                    case 3: EP3_SOF_Callback(); break;
                    case 4: EP4_SOF_Callback(); break;
                    default: break;
                }
            }
                break;
            case UIS_TOKEN_IN:
            {//SDCC will take IRAM if array of function pointer is used.
                switch (callIndex) {
                    case 0: EP0_IN_Callback(); break;
                    case 1: EP1_IN_Callback(); break;
                    case 2: EP2_IN_Callback(); break;
                    case 3: EP3_IN_Callback(); break;
                    case 4: EP4_IN_Callback(); break;
                    default: break;
                }
            }
                break;
            case UIS_TOKEN_SETUP:
            {//SDCC will take IRAM if array of function pointer is used.
                switch (callIndex) {
                    case 0: EP0_SETUP_Callback(); break;
                    case 1: EP1_SETUP_Callback(); break;
                    case 2: EP2_SETUP_Callback(); break;
                    case 3: EP3_SETUP_Callback(); break;
                    case 4: EP4_SETUP_Callback(); break;
                    default: break;
                }
            }
                break;
        }
        
        UIF_TRANSFER = 0;                                                     // Clear interrupt flag
    }
    
    // Device mode USB bus reset
    if(UIF_BUS_RST) {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
        UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
        UEP4_CTRL = UEP_T_RES_NAK | UEP_R_RES_ACK;  //bUEP_AUTO_TOG only work for endpoint 1,2,3
        
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                        // Clear interrupt flag
        
        resetCDCParameters();
    }
    
    // USB bus suspend / wake up
    if (UIF_SUSPEND) {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND ) {                    // Suspend
            
            //while ( XBUS_AUX & bUART0_TX );                    // Wait for Tx
            //SAFE_MOD = 0x55;
            //SAFE_MOD = 0xAA;
            //WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;    // Wake up by USB or RxD0
            //PCON |= PD;                                                            // Chip sleep
            //SAFE_MOD = 0x55;
            //SAFE_MOD = 0xAA;
            //WAKE_CTRL = 0x00;
            
        } else {    // Unexpected interrupt, not supposed to happen !
            USB_INT_FG = 0xFF;        // Clear interrupt flag
        }
    }
}
#pragma restore

void USBDeviceCfg()
{
    USB_CTRL = 0x00;                                                           //??USB?????
    USB_CTRL &= ~bUC_HOST_MODE;                                                //?????????
    USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                    //USB?????????,?????????????????NAK
    USB_DEV_AD = 0x00;                                                         //???????
    //     USB_CTRL |= bUC_LOW_SPEED;
    //     UDEV_CTRL |= bUD_LOW_SPEED;                                                //????1.5M??
    USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED;                                             //????12M???????
    UDEV_CTRL = bUD_PD_DIS;  // ??DP/DM????
    UDEV_CTRL |= bUD_PORT_EN;                                                  //??????
}

void USBDeviceIntCfg()
{
    USB_INT_EN |= bUIE_SUSPEND;                                               //????????
    USB_INT_EN |= bUIE_TRANSFER;                                              //??USB??????
    USB_INT_EN |= bUIE_BUS_RST;                                               //??????USB??????
    USB_INT_FG |= 0x1F;                                                       //?????
    IE_USB = 1;                                                               //??USB??
    EA = 1;                                                                   //???????
}

void USBDeviceEndPointCfg()
{
    UEP1_DMA = (uint16_t) Ep1Buffer;                                                      //??1 ????????
    UEP2_DMA = (uint16_t) Ep2Buffer;                                                      //??2 ??????
    UEP2_3_MOD = 0x0C;                                                         //??2 double buffer
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                //??2??????????IN????NAK
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;        //??3??????????IN????NAK?OUT??ACK
    
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                //??1??????????IN????NAK
    UEP0_DMA = (uint16_t) Ep0Buffer;                                                      //??0??????
    UEP4_1_MOD = 0X40;                                                         //endpoint1 TX enable
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                //?????OUT????ACK?IN????NAK
}

