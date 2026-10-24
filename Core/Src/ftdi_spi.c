/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi_spi.c
* Description  : SPI2硬件移位引擎实现
*               硬件连接是天然的: PB13=SCK PB14=MISO PB15=MOSI 恰好是TCK/TDO/TDI
*               JTAG时序与SPI模式0兼容: TDI在下降沿变化(上升沿被FPGA采样),
*               TDO在下降沿更新(我们在上升沿采样), 全双工正好对上
*               DMA: SPI2_RX=DMA1通道4, SPI2_TX=DMA1通道5(查RM映射表)
*******************************************************************************/
#include "ch32v30x.h"
#include "ch32v30x_spi.h"
#include "ftdi_spi.h"
#include <string.h>

static __attribute__((aligned(4))) uint8_t spi_bounce[FTDI_SPI_BOUNCE_SZ];
static uint8_t spi_pin_state;   // 0=GPIO模式 1=SPI模式

// DMA标志位: 通道n的TCIF在INTFR的4*(n-1)+1位, 通道4=bit13, 通道5=bit17
#define DMA_TC4_IF   (1u << 13)
#define DMA_TC5_IF   (1u << 17)
#define DMA_TC4_ICR  (1u << 13)
#define DMA_TC5_ICR  (1u << 17)

void ftdi_spi_init(void)
{
    // 144MHz HCLK / 2 = 72MHz APB1; SPI / 4 gives fixed 18MHz TCK.
    RCC->APB1PCENR |= RCC_APB1Periph_SPI2;
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    // SPI2: master, software NSS, mode 0, 8-bit, APB1 / 4 (18MHz).
    // 每次传输前再按命令改LSBFE, SPE在传输时开
    SPI2->CTLR1 = SPI_CTLR1_MSTR | SPI_CTLR1_SSM | SPI_CTLR1_SSI | SPI_BaudRatePrescaler_4;
    SPI2->CTLR2 = SPI_CTLR2_TXDMAEN | SPI_CTLR2_RXDMAEN;
    SPI2->CRCR = 0;

    // DMA通道固定部分: 字节宽度/自增/高优先级
    // CH4 = SPI2_RX 外设->存储器(DIR=0)
    DMA1_Channel4->PADDR = (uint32_t)&SPI2->DATAR;
    DMA1_Channel4->CFGR = (3u << 12)      // PL=高优先级
                          | (1u << 7);    // MINC 存储器自增(WCH位布局, bit7)
    // CH5 = SPI2_TX 存储器->外设(DIR=bit4=1)
    DMA1_Channel5->PADDR = (uint32_t)&SPI2->DATAR;
    DMA1_Channel5->CFGR = (3u << 12) | (1u << 7) | (1u << 4);  // PL高/MINC/DIR=存->外设
}

void ftdi_spi_engage(uint8_t lsb_first)
{
    if (!spi_pin_state) {
        spi_pin_state = 1;
        // PB13/PB15: alternate-function push-pull; PB14: pulled-up input.
        uint32_t v = GPIOB->CFGHR;
        v = (v & ~(0xFu << 20)) | (0xBu << 20);
        v = (v & ~(0xFu << 24)) | (0x8u << 24);
        v = (v & ~(0xFu << 28)) | (0xBu << 28);
        GPIOB->CFGHR = v;
    }
    SPI2->CTLR1 = (SPI2->CTLR1 & ~SPI_CTLR1_LSBFIRST) |
                  (lsb_first ? SPI_CTLR1_LSBFIRST : 0);
}

void ftdi_spi_release(void)
{
    if (!spi_pin_state) return;
    spi_pin_state = 0;
    SPI2->CTLR1 &= ~SPI_CTLR1_SPE;
    // Prime the GPIO latch while SPI still owns TCK. Otherwise restoring
    // GPIO with a stale high latch creates an extra rising edge.
    GPIOB->BSHR = ((uint32_t)1 << 13) << 16;
    // 切回GPIO: PB13(TCK)/PB15(TDI)推挽输出, PB14(TDO)输入上拉
    {
        uint32_t v = GPIOB->CFGHR;
        v = (v & ~(0xFu << 20)) | (0x3u << 20);   // PB13: 推挽50M
        v = (v & ~(0xFu << 24)) | (0x8u << 24);   // PB14: 输入上拉
        v = (v & ~(0xFu << 28)) | (0x3u << 28);   // PB15: 推挽50M
        GPIOB->CFGHR = v;
        GPIOB->BSHR = ((uint32_t)1 << 13) << 16;   // TCK(PB13)回空闲低电平
    }
}

int ftdi_spi_shift(const uint8_t *tx, uint16_t n)
{
    uint32_t tmo;

    if (n == 0) return 0;
    if (n > FTDI_SPI_BOUNCE_SZ) n = FTDI_SPI_BOUNCE_SZ;

    SPI2->CTLR1 &= ~SPI_CTLR1_SPE;

    // 清历史标志, 配通道
    DMA1->INTFCR = DMA_TC4_ICR | DMA_TC5_ICR;
    DMA1_Channel4->MADDR = (uint32_t)spi_bounce;
    DMA1_Channel4->CNTR = n;
    DMA1_Channel5->MADDR = (uint32_t)tx;
    DMA1_Channel5->CNTR = n;
    DMA1_Channel4->CFGR |= 1;   // EN
    DMA1_Channel5->CFGR |= 1;   // EN

    SPI2->CTLR1 |= SPI_CTLR1_SPE;

    // 等两个通道传输完成(18MHz下512字节约228us, 预留10倍超时)
    tmo = 0;
    while ((DMA1->INTFR & (DMA_TC4_IF | DMA_TC5_IF)) != (DMA_TC4_IF | DMA_TC5_IF)) {
        if (++tmo > 40000000u) {
            // 超时: 全关, 报错
            DMA1_Channel4->CFGR &= ~1;
            DMA1_Channel5->CFGR &= ~1;
            SPI2->CTLR1 &= ~SPI_CTLR1_SPE;
            return -1;
        }
    }
    // 等SPI把最后一位移完
    while (SPI2->STATR & SPI_I2S_FLAG_BSY) ;
    SPI2->CTLR1 &= ~SPI_CTLR1_SPE;

    DMA1_Channel4->CFGR &= ~1;
    DMA1_Channel5->CFGR &= ~1;
    DMA1->INTFCR = DMA_TC4_ICR | DMA_TC5_ICR;
    return 0;
}

uint8_t *ftdi_spi_bounce(void)
{
    return spi_bounce;
}
