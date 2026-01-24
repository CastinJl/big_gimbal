//ws2812 by zyx
#ifndef RGB_H
#define RGB_H


#include <stdint.h>

//            编码 0 : 11000000
#define CODE_0		0xC0
//            编码 1 : 11111000
#define CODE_1		0xF8
/*ws2812b灯珠数量*/
#define WS2812B_AMOUNT		8

typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} tWs2812bCache_TypeDef;

extern tWs2812bCache_TypeDef gWs2812bDat[WS2812B_AMOUNT];

void WS2812b_Set(uint16_t Ws2b812b_NUM, uint8_t r,uint8_t g,uint8_t b);
void WS2812B_Task(void);
void move_Front(void);

#endif
