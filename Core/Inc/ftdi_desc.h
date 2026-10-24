/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi_desc.h
* Description  : FT2232H风格的USB描述符
*******************************************************************************/
#ifndef __FTDI_DESC_H
#define __FTDI_DESC_H

#include <stdint.h>

// speed: 1=高速 0=全速; 两种速度都报告USBHS控制器的5接口布局
// 返回0=找到描述符, *pptr/[*plen]给出内容和总长; 返回0xFF=不支持
int ftdi_get_descriptor(uint8_t speed, uint16_t wValue,
                        const uint8_t **pptr, uint16_t *plen);

// 假EEPROM(93C56风格, 128个16位字), 给libftdi/ftx-prog这类读EEPROM的工具看
uint16_t ftdi_eeprom_word(uint16_t addr);

#endif /* __FTDI_DESC_H */
