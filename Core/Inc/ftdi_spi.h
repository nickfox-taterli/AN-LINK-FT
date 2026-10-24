/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi_spi.h
* Description  : SPI2硬件移位引擎
*               PB13=SPI2_SCK(TCK) PB14=SPI2_MISO(TDO) PB15=SPI2_MOSI(TDI)
*               大批量数据移位命令交给SPI+DMA, TCK最高18MHz且零CPU占用
*               仅支持SPI模式0语义: 负沿写/正沿读(FTDI默认组合)
*******************************************************************************/
#ifndef __FTDI_SPI_H
#define __FTDI_SPI_H

#include <stdint.h>

#define FTDI_SPI_BOUNCE_SZ   512

void ftdi_spi_init(void);
// 引脚复用切换: 1=切到SPI(占用PB13/14/15), 0=切回GPIO(TMS等位操作需要)
void ftdi_spi_engage(uint8_t lsb_first);
void ftdi_spi_release(void);
// 全双工移位n字节, TX发tx, RX收进内部接缓冲; 返回0成功
int  ftdi_spi_shift(const uint8_t *tx, uint16_t n);
// 接缓冲(存SPI收到的TDO数据)
uint8_t *ftdi_spi_bounce(void);

#endif /* __FTDI_SPI_H */
