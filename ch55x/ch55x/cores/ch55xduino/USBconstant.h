#ifndef __CONST_DATA_H__
#define __CONST_DATA_H__

#include <stdint.h>
#include "include/ch554.h"
#include "include/ch554_usb.h"

#define  SET_LINE_CODING                0X20            // Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING                0X21            // This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE         0X22            // This request generates RS-232/V.24 style control signals.


extern __code uint8_t DevDesc[];
extern __code uint8_t CfgDesc[];
extern __code uint8_t LangDes[];
extern __code uint8_t SerDes[];
extern __code uint8_t Prod_Des[];
extern __code uint8_t CDC_Des[];
extern __code uint8_t WEBUSB_Des[];
extern __code uint8_t Manuf_Des[];



extern __code uint16_t DevDescLen;
extern __code uint16_t CfgDescLen;
extern __code uint16_t LangDesLen;
extern __code uint16_t SerDesLen;
extern __code uint16_t Prod_DesLen;
extern __code uint16_t CDC_DesLen;
extern __code uint16_t Manuf_DesLen;

#endif
