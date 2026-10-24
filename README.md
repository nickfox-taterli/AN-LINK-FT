# 安路AN-LINK-FT开源实现

MCU是CH32V307VCT6,已在DR1M90GEG484实板跑通,环境TD/FD 2026.2.

## USB 身份与接口

- VID=0x0403 PID=0x6042, bcdDevice=0x0700, iSerial=ALFT9CF0DA —— 与真安路线一致,安路工具按 VID/PID 直接识别; libftdi 认成 FT2232H(60MHz 时钟表)
- USBHS 480M 高速
- 接口分配与端点:

| USB 接口 | 功能 | 端点 | 对应硬件 |
|---|---|---|---|
| 0 (A) | JTAG (MPSSE) | EP1 IN / EP2 OUT | GPIOB + SPI2+DMA, TCK 固定 18MHz (人工修改ftdi_spi_init可降速.) |
| 1 (B) | USB TTL 串口 | EP3 IN / EP4 OUT | USART1 |
| 2 (C) | USB TTL 串口 | EP5 IN / EP6 OUT | UART6 |
| 3 (D) | USB TTL 串口 | EP7 IN / EP8 OUT | UART7 |
| 4 (E) | USB TTL 串口 | EP9 IN / EP10 OUT | UART8 |

Linux下整机被ftdi_sio认领时串口直接成/dev/ttyUSBx,JTAG 要用libftdi/TD时需先解绑接口0的ftdi_sio,在Windows下要用USB Driver Tool把接口0绑定成WinUSB驱动.

## 接线

### JTAG(接口0)

| 信号 | MCU 引脚 |
|------|----------|
| TCK  | PB13     |
| TMS  | PB12     |
| TDI  | PB15     |
| TDO  | PB14     |

### USB TTL 串口(接口1~4)

| 接口 | UART | TX | RX |
|------|------|----|----|
| B(接口1) | USART1 | PA9  | PA10 |
| C(接口2) | UART6  | PC0  | PC1  |
| D(接口3) | UART7  | PC2  | PC3  |
| E(接口4) | UART8  | PC4  | PC5  |

上电默认 115200 8N1; 支持 300~6000000 波特率、7/8 数据位、奇/偶校验、1/1.5/2 停止位(FTDI 编码反算, 串口工具直接改即可)。

### 其他 LED

- LED1(PA15) = JTAG 活动灯: 有 MPSSE 流量点亮, 空闲熄灭
- LED2(PB4) = 心跳灯: 固件活着就慢闪

上电后LED2慢闪即固件存活.

## 实测

目标:DR1M90GEG484 I2, PLJTAG 四 TAP 链(dummy 5 + ARM DAP 4 + PS TAP 4 + PL FPGA 8, 总 IR 21 位,使用 TD/FD 2026.2 原厂工具:

| 场景 | 结果 |
|------|------|
| TD 原生下载 4,052,153 字节 bit | 7.13 (Linux实测) 7.36s (Windows实测) |
| TD 完整 PL ID / DONE / 随机长扫描 | 全部通过(512~65537 字节 BYPASS 全位比较) |
| FD 原生 pld load | 1.87s |
| FD 双 ARM 核暂停 / 读 PC / 恢复 | 两核均通过 |
| FD 读取 64KiB OCM | 0.490s, 与优化前 SHA256 一致 |
| FD 随机 BYPASS 扫描 | 1~524293 位全通过(位尾/DMA块/USB包边界覆盖) |

## 源码结构

```
CMakeLists.txt             # 构建(默认-O3), 产物 chftdi.bin
Link.ld                    # 256K code flash + 64K RAM
Core/Inc/
  ftdi.h                   # 公共定义: 引脚/环形缓冲/MPSSE状态机/端口结构
  ftdi_desc.h
Core/Src/
  main.c                   # 主循环: 5个USBHS端口任务轮询 + LED
  usbhs_ftdi.c             # 自写USBHS栈(无TinyUSB), 5接口 EP1~10
  ftdi_mpsse.c             # MPSSE引擎 + JTAG位泵(GPIO路径)
  ftdi_spi.c               # SPI2+DMA1 CH4/5 固定18MHz高速字节移位
  ftdi_serial.c            # 四路USB TTL(USART1/UART6/UART7/UART8)
  ftdi_desc.c              # FT4232H描述符(统一0403:6042身份)
  syscalls.c               # newlib裸机桩(printf必须)
  ch32v30x_it.c            # 中断向量(USBHS=IRQn 85, 必须D8C启动文件)
Core/Startup/              # startup_ch32v30x_D8C.S(EVT旧向量表不可用)
Drivers/                   # WCH标准外设库子集(自包含)
```