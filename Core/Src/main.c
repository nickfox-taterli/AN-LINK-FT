/********************************** (C) COPYRIGHT *******************************
* File Name    : main.c
* Description  : 安路下载器模拟
*               A通道 = JTAG(MPSSE): TCK=PB13 TDI=PB15 TDO=PB14 TMS=PB12
*               B/C/D通道 = 三路USB TTL:
*                 B: PA9=TX PA10=RX   C: PC0=TX PC1=RX   D: PC2=TX PC3=RX
*               USB2口(PB6/PB7)是USBHS高速(5接口)
*               LED1(PA15)=JTAG活动灯, LED2(PB4)=心跳灯
*******************************************************************************/
#include "debug.h"
#include "ftdi.h"
#include "ftdi_spi.h"

extern void usbhs_ftdi_init(void);
extern volatile uint8_t g_ftdi_active;

#define LED_ACT_ON()   GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define LED_ACT_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define LED_HB_TOGGLE() GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) ? \
                        GPIO_ResetBits(GPIOB, GPIO_Pin_4) : GPIO_SetBits(GPIOB, GPIO_Pin_4)

static void led_init(void)
{
    GPIO_InitTypeDef gi;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    gi.GPIO_Pin = GPIO_Pin_15;   // LED1
    gi.GPIO_Mode = GPIO_Mode_Out_PP;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gi);

    gi.GPIO_Pin = GPIO_Pin_4;    // LED2
    GPIO_Init(GPIOB, &gi);

    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}

int main(void)
{
    uint32_t loop = 0;

    SystemCoreClockUpdate();
    Delay_Init();   // 只为USB初始化里的Delay_Us, 不再要调试串口

    ftdi_jtag_gpio_init();
    ftdi_spi_init();
    led_init();
    ftdi_serial_init();

    usbhs_ftdi_init();

    // SysTick转自由计数(HCLK/8), 给空闲补状态包提供毫秒时基
    SysTick->CNT = 0;
    SysTick->CMP = 0xFFFFFFFFFFFFFFFFULL;
    SysTick->CTLR = (1 << 0) | (1 << 5);

    while (1) {
        int i;

        ftdi_port_task(&g_ftdi_hs);                       // HS A: JTAG
        for (i = 0; i < FT_SER_CH; i++) ftdi_port_task(&g_ser_hs[i]);   // HS B/C/D/E

        // 活动灯: 有MPSSE流量时点亮, 衰减计数清零后熄灭
        if (g_ftdi_active) {
            g_ftdi_active--;
            LED_ACT_ON();
        } else {
            LED_ACT_OFF();
        }

        // 心跳灯: 空转计数翻转, 频率不精确, 能看出固件活着就行
        if ((++loop & 0x7FFFF) == 0) {
            LED_HB_TOGGLE();
        }
    }
}
