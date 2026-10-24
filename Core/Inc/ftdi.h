/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi.h
* Description  : FTDI FT2232H 模拟层公共定义
*               A通道=MPSSE(JTAG), B通道=USB TTL串口(USART1 PA9/PA10)
*               仅USBHS口工作, USB设备仍提供全速fallback描述符
*******************************************************************************/
#ifndef __FTDI_H
#define __FTDI_H

#include "ch32v30x.h"

// ---- JTAG引脚, 全在GPIOB上, 方便BSRR原子操作 ----
// 按FTDI线序: ADBUS0=TCK, ADBUS1=TDI, ADBUS2=TDO, ADBUS3=TMS
#define FT_PIN_TCK    (1u << 13)   // PB13
#define FT_PIN_TDI    (1u << 15)   // PB15
#define FT_PIN_TDO    (1u << 14)   // PB14, 只做输入
#define FT_PIN_TMS    (1u << 12)   // PB12

// BSRR写法: 置位=bitn, 清零=bitn<<16
#define FT_TCK_LO     ((uint32_t)FT_PIN_TCK << 16)
#define FT_TCK_HI     ((uint32_t)FT_PIN_TCK)
#define FT_TDI_SET    ((uint32_t)FT_PIN_TDI)
#define FT_TDI_CLR    ((uint32_t)FT_PIN_TDI << 16)
#define FT_TMS_SET    ((uint32_t)FT_PIN_TMS)
#define FT_TMS_CLR    ((uint32_t)FT_PIN_TMS << 16)
#define FT_TDO_MSK    ((uint32_t)FT_PIN_TDO)

// OUT方向(主机发给设备)环形块数
#define FT_BLK_NUM    8
// IN方向(设备回给主机)环形缓冲长度
#define FT_TX_RING    1024

// MPSSE命令流解析状态
enum {
    MS_IDLE = 0,              // 等命令字节
    MS_LEN_LO, MS_LEN_HI,     // 字节/位命令的16位长度
    MS_DATA,                  // 数据字节流
    MS_TMS_LEN, MS_TMS_DAT,   // TMS命令: [长度-1][数据]
    MS_GPIO_VAL, MS_GPIO_DIR, // 0x80/0x82的[值][方向]
    MS_DIV_LO, MS_DIV_HI,     // 0x86分频值
    MS_CKB_LEN,               // 0x8E空转位数
    MS_CKBYTES_LO, MS_CKBYTES_HI, // 0x8F空转字节数
    MS_ERR_ECHO,              // 非法命令: 回显0xFA+命令
};

// SIO_RESET的wValue含义(libftdi: 0=复位 1=清RX 2=清TX)
#define FT_PURGE_SIO    0
#define FT_PURGE_RX     1
#define FT_PURGE_TX     2

typedef struct ftdi_port {
    // OUT流: USB中断生产, 主循环消费
    uint8_t           *rxblk[FT_BLK_NUM];  // 每块大小 = rxblksz
    uint16_t           rxblksz;
    volatile uint16_t  rxlen[FT_BLK_NUM];
    volatile uint8_t   rx_load;            // USB中断即将写入的块号
    uint8_t            rx_deal;            // 主循环待处理的块号
    uint16_t           blk_pos;            // 当前块内消费到的位置(支持块内部分消费)
    volatile uint8_t   rx_stalled;         // 环满, OUT端点被NAK住了
    // IN流: 主循环生产, USB中断消费
    uint8_t            txring[FT_TX_RING];
    volatile uint16_t  tx_head, tx_tail;
    uint16_t           maxpack;            // IN端点最大包长
    volatile uint8_t   in_busy;
    volatile uint16_t  in_sent;            // 本次IN传输字节数
    // MPSSE解析状态
    uint8_t  st;
    uint8_t  cmd;
    uint32_t cnt;          // 剩余数据字节数(0xFFFF+1=65536, 必须32位)
    uint8_t  gpio_stage;   // 0x80/0x82先收到val暂存
    // 运行配置
    uint8_t  is_serial;    // 1=串口通道实例(走USB-TTL路径而不是MPSSE)
    uint8_t  ser_ch;       // 串口通道号(0=B 1=C 2=D)
    uint8_t  latency;      // 延时定时器(ms), 空闲补状态包的周期
    uint8_t  loopback;     // 环回模式
    uint8_t  div5;         // 5分频使能(2232系列上电默认开)
    uint8_t  phase3;       // 3相时钟
    uint16_t divisor;      // TCK分频值
    uint8_t  spi_ok;       // 1=固定高速模式, 兼容的字节移位交给SPI2硬件
    uint8_t  bitmode;      // 0=串口模式 2=MPSSE (串口实例恒为1)
    uint8_t  low_val, low_dir, high_val, high_dir;
    uint32_t dly;          // 每半周期延时滴答数(0=全速)
    // 延迟执行的请求(USB中断置位, 主循环在块边界应用, 避免并发踩状态)
    volatile uint8_t req;  // bit0=清缓冲 bit1=总线复位 bit2=进MPSSE
    volatile uint8_t want_status;  // 要发一个纯modem状态包(进MPSSE后首次读)
    uint32_t last_tx_tick;         // 上次发包的时刻, 空闲补状态包用
    // USB栈提供的回调
    void (*kick_in)(struct ftdi_port *p);   // 有数据了, 试着启动一次IN传输
    void (*rearm_out)(struct ftdi_port *p); // OUT环腾出空位后重新ACK
} ftdi_port_t;

// 接口分配: 0=JTAG(MPSSE), 其余全是USB TTL串口
// USBHS口5接口(A=JTAG, B/C/D/E=串口)
// 统一身份: 0403:6042 安路USB-JTAG Cable
#define FT_SER_CH    4   // 硬件串口: ch0=USART1(PA9/10) ch1=UART6(PC0/1) ch2=UART7(PC2/3) ch3=UART8(PC4/5)

// USBHS端口实例(在usbhs_ftdi.c里定义)
extern ftdi_port_t g_ftdi_hs;        // USBHS A通道: MPSSE JTAG (EP1 IN/EP2 OUT)
extern ftdi_port_t g_ser_hs[FT_SER_CH];   // USBHS B/C/D/E: TTL (EP3/4 ... EP9/10)

void    ftdi_jtag_gpio_init(void);
void    ftdi_port_init(ftdi_port_t *p, uint8_t **blocks, uint16_t blksz,
                       uint16_t maxpack,
                       void (*kick)(ftdi_port_t *), void (*rearm)(ftdi_port_t *));
void    ftdi_port_reset(ftdi_port_t *p);        // USB总线复位时清状态
void    ftdi_port_task(ftdi_port_t *p);         // 主循环里跑(A=MPSSE引擎, B=串口搬运)
void    ftdi_port_purge(ftdi_port_t *p, uint8_t which);

// EP0厂商请求处理(A通道MPSSE语义), 返回0=已处理, 0xFF=不支持要STALL
// 入参plen为wLength, 出参plen为要回的数据长度(仅IN方向)
int     ftdi_ep0_vendor(ftdi_port_t *p, uint8_t req, uint16_t val, uint16_t idx,
                        uint16_t *plen, uint8_t *buf);

uint8_t ftdi_read_adbus(void);                  // 读ADBUS0..3实际电平
void    ftdi_set_adbus(uint8_t val, uint8_t dir); // SET_BITS_LOW落到物理脚

// ---- USB TTL(B/C/D通道) ----
void    ftdi_serial_init(void);   // USART1(PA9/10) + UART6(PC0/1) + UART7(PC2/3)
void    ftdi_serial_task(ftdi_port_t *p);   // 串口通道的OUT/IN搬运
int     ftdi_serial_ep0(ftdi_port_t *p, uint8_t req, uint16_t val, uint16_t idx,
                        uint16_t *plen, uint8_t *buf);  // 串口通道EP0厂商请求

#endif /* __FTDI_H */
