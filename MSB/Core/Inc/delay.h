#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f4xx.h"

void delay_nms(uint32_t nms){
    nms = nms * 1000 - 2;
    uint16_t i = 0;
    while(i<=nms) i++;
}
#endif





























