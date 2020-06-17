#ifndef WiringPrivate_h
#define WiringPrivate_h

#include <stdint.h>
#include <stdio.h>
#if defined(__CH559__)
 #include "include/ch559.h"
 #include "include/ch559_usb.h"
#else
  #include "include/ch554.h"
  #include "include/ch554_usb.h"
#endif

#include "Arduino.h"


typedef void (*voidFuncPtr)(void);

#define EXTERNAL_INT_0 0
#define EXTERNAL_INT_1 1

#define EXTERNAL_NUM_INTERRUPTS 2

#endif