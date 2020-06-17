#include <stdint.h>
#include <stdbool.h>
#include "include/ch554.h"
#include "include/ch554_usb.h"

extern __xdata __at (EP0_ADDR) uint8_t  Ep0Buffer[];
extern __xdata __at (EP2_ADDR) uint8_t  Ep2Buffer[];

#define LINE_CODEING_SIZE 7
__xdata uint8_t LineCoding[LINE_CODEING_SIZE]={0x00,0xe1,0x00,0x00,0x00,0x00,0x08};   //初始化波特率为57600，1停止位，无校验，8数据位。

volatile __xdata uint8_t USBByteCountEP2 = 0;      //代表USB端点接收到的数据
volatile __xdata uint8_t USBBufOutPointEP2 = 0;    //取数据指针

volatile __xdata uint8_t UpPoint2_Busy  = 0;   //上传端点是否忙标志
volatile __xdata uint8_t controlLineState = 0;

__xdata uint8_t usbWritePointer = 0;

typedef void( *pTaskFn)( void );

void mDelayuS( uint16_t n );
void mDelaymS( uint16_t n );

void resetCDCParameters(){

    USBByteCountEP2 = 0;       //USB端点收到的长度
    UpPoint2_Busy = 0;
}

void setLineCodingHandler(){
    for (uint8_t i=0;i<((LINE_CODEING_SIZE<=USB_RX_LEN)?LINE_CODEING_SIZE:USB_RX_LEN);i++){
        LineCoding[i] = Ep0Buffer[i];
    }
    
    //!!!!!Config_Uart0(LineCoding);
}

uint16_t getLineCodingHandler(){
    uint16_t returnLen;

    returnLen = LINE_CODEING_SIZE;
    for (uint8_t i=0;i<returnLen;i++){
        Ep0Buffer[i] = LineCoding[i];
    }

    return returnLen;
}

void setControlLineStateHandler(){
    controlLineState = Ep0Buffer[2];

    // We check DTR state to determine if host port is open (bit 0 of lineState).
    if ( ((controlLineState & 0x01) == 0) && (*((__xdata uint32_t *)LineCoding) == 1200) ){ //both linecoding and sdcc are little-endian
        pTaskFn tasksArr[1];
        USB_CTRL = 0;
        EA = 0;                                                                    //关闭总中断，必加
        tasksArr[0] = (pTaskFn)0x3800;
        mDelaymS( 100 );     
        (tasksArr[0])( );                                                          //跳至BOOT升级程序,使用ISP工具升级
        while(1);
    }
    
}

bool USBSerial(){
    bool result = false;
	if (controlLineState > 0) 
		result = true;
	//mDelaymS(10); not doing it for now
	return result;
}


void USBSerial_flush(void){
    if (!UpPoint2_Busy && usbWritePointer>0){
        UEP2_T_LEN = usbWritePointer;                                                    //预使用发送长度一定要清空
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //应答ACK
        UpPoint2_Busy = 1;
        usbWritePointer = 0;
    }
}

uint8_t USBSerial_write(char c){  //3 bytes generic pointer
    uint16_t waitWriteCount;
    if (controlLineState > 0) {
        while (true){
            waitWriteCount = 0;
            while (UpPoint2_Busy){//wait for 250ms or give up, on my mac it takes about 256us
                waitWriteCount++;
                mDelayuS(5);   
                if (waitWriteCount>=50000) return 0;
            }
            if (usbWritePointer<MAX_PACKET_SIZE){
                Ep2Buffer[MAX_PACKET_SIZE+usbWritePointer] = c;
                usbWritePointer++;
                return 1;
            }else{
                USBSerial_flush();  //go back to first while
            }
        }
    }
    return 0;
}

uint8_t USBSerial_print_n(uint8_t * __xdata buf, __xdata int len){  //3 bytes generic pointer, not using USBSerial_write for a bit efficiency
    uint16_t waitWriteCount;
    if (controlLineState > 0) {
        while (len>0){
            waitWriteCount = 0;
            while (UpPoint2_Busy){//wait for 250ms or give up, on my mac it takes about 256us
                waitWriteCount++;
                mDelayuS(5);   
                if (waitWriteCount>=50000) return 0;
            }
            while (len>0){
                if (usbWritePointer<MAX_PACKET_SIZE){
                    Ep2Buffer[MAX_PACKET_SIZE+usbWritePointer] = *buf++;
                    usbWritePointer++;
                    len--;
                }else{
                    USBSerial_flush();  //go back to first while
                    break;
                }
            }
        }
    }
    return 0;
}

uint8_t USBSerial_available(){
    return USBByteCountEP2;
}

char USBSerial_read(){
    if(USBByteCountEP2==0) return 0;
    char data = Ep2Buffer[USBBufOutPointEP2];
    USBBufOutPointEP2++;
    USBByteCountEP2--;
    if(USBByteCountEP2==0) {
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
    }
    return data;
}

void USB_EP2_IN(){
    UEP2_T_LEN = 0;                                                    //预使用发送长度一定要清空
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
    UpPoint2_Busy = 0;                                                  //清除忙标志
}

void USB_EP2_OUT(){
    if ( U_TOG_OK )                                                     // 不同步的数据包将丢弃
    {
        USBByteCountEP2 = USB_RX_LEN;
        USBBufOutPointEP2 = 0;                                             //取数据指针复位
        if (USBByteCountEP2)    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;       //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
    }
}