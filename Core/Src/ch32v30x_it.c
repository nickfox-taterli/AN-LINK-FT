/********************************** (C) COPYRIGHT *******************************
* File Name    : ch32v30x_it.c
* Description  : 中断服务程序收集, USB相关IRQ在各usb_xx_ftdi.c里
*******************************************************************************/
#include "ch32v30x_it.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {
    }
}
