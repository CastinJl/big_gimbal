//ws2812 by zyx
#include "RGB.h"
#include "main.h"
#include "spi.h"
#include "bsp_spi.h"

//灯条显存SPI数据缓存
uint8_t gWs2812bDat_SPI[WS2812B_AMOUNT * 24] = {0};
//灯条显存
tWs2812bCache_TypeDef gWs2812bDat[WS2812B_AMOUNT] = {

//R    G      B
        0XFF, 0X00, 0X00,	//0
        0X00, 0XFF, 0X00,	//1
        0X00, 0X00, 0XFF,	//2
        0X00, 0XFF, 0XFF,	//3
        0XFF, 0X00, 0XFF,	//4
        0XFF, 0XFF, 0X00,	//5
        0XFF, 0XFF, 0XFF,	//6
        0X00, 0X00, 0X00,	//7
};

void WS2812b_Set(uint16_t Ws2b812b_NUM, uint8_t r,uint8_t g,uint8_t b)
{
    uint8_t *pR = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24 + 8];
    uint8_t *pG = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24];
    uint8_t *pB = &gWs2812bDat_SPI[(Ws2b812b_NUM) * 24 + 16];

    for(uint8_t i = 0; i <  8; i++) {
        if(g & 0x80) {
            *pG = CODE_1;
        }
        else {
            *pG = CODE_0;
        }
        if(r & 0x80) {
            *pR = CODE_1;
        }
        else {
            *pR = CODE_0;
        }
        if(b & 0x80) {
            *pB = CODE_1;
        }
        else {
            *pB = CODE_0;
        }
        r <<= 1;
        g <<= 1;
        b <<= 1;
        pR++;
        pG++;
        pB++;
    }
}
void WS2812B_Task(void)
{
    uint8_t dat = 0;

    //将gWs2812bDat数据解析成SPI数据
    for(uint8_t iLED = 0; iLED < WS2812B_AMOUNT; iLED++)
    {
        WS2812b_Set(iLED, gWs2812bDat[iLED].R, gWs2812bDat[iLED].G, gWs2812bDat[iLED].B);
    }
    //总线输出数据
    HAL_SPI_Transmit(&hspi1, gWs2812bDat_SPI, sizeof(gWs2812bDat_SPI),0XFFFF);
    //使总线输出低电平
    HAL_SPI_Transmit(&hspi1, &dat, 1,0XFFFF);
    //帧信号：一个大于50us的低电平
    HAL_Delay(1);
}
void move_Front()
{
    uint8_t i;
    uint8_t temp[3];
    temp[0] = gWs2812bDat[0].R;
    temp[1] = gWs2812bDat[0].G;
    temp[2] = gWs2812bDat[0].B;
    for (i = 0; i < WS2812B_AMOUNT-1; i++)
    {
        gWs2812bDat[i].R = gWs2812bDat[i+1].R;
        gWs2812bDat[i].G = gWs2812bDat[i+1].G;
        gWs2812bDat[i].B = gWs2812bDat[i+1].B;
    }
    gWs2812bDat[7].R = temp[0];
    gWs2812bDat[7].G = temp[1];
    gWs2812bDat[7].B = temp[2];
}

