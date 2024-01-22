#pragma once
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "dma.h"
#include "i2c.h"


void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif

#endif