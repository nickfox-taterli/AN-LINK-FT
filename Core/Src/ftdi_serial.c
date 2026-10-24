/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi_serial.c
* Description  : B/C/D/E四路USB TTL串口(JTAG外的接口全是串口)
*               B: USART1  PA9=TX  PA10=RX
*               C: UART6   PC0=TX  PC1=RX
*               D: UART7   PC2=TX  PC3=RX
*               E: UART8   PC4=TX  PC5=RX
*               每个USBHS串口实例使用独立收发环
*               Windows/Linux的FTDI驱动打开COM口后, SET_BAUDRATE/SET_DATA等
*               控制请求发到对应通道(wIndex低字节=通道号+1)
*******************************************************************************/
#include "ftdi.h"
#include <string.h>

#define SER_RING    512    // 每通道串口收发环形缓冲, 2的幂

typedef struct {
    USART_TypeDef *uart;
    // 收发环: rx=串口→USB, tx=USB→串口
    // rx生产者=串口中断, 消费者=task(关中断抢占式领取)
    // tx生产者=task(关中断保护), 消费者=串口发送中断
    uint8_t  rx[SER_RING];
    uint8_t  tx[SER_RING];
    volatile uint16_t rxh, rxt, txh, txt;
    // 线格式状态
    uint32_t baud;
    uint8_t  bits, par, stop;
} ser_ch_t;

static ser_ch_t g_ser[FT_SER_CH];

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART6_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void UART7_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void UART8_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));

// ---- 波特率解码: 反算libftdi/FTDI驱动的编码 ----
// wValue = 编码分频低16位(整数11位 + 小数3位在bit14..16)
// wIndex bit9 = 编码bit17, 即CLK/10开关; wIndex低字节 = 通道号
static uint32_t serial_decode_baud(uint16_t value, uint16_t index)
{
    static const uint8_t frac_inv[8] = {0, 4, 2, 1, 3, 5, 6, 7};  // frac_code反查
    uint32_t enc = value | (((uint32_t)(index & 0xFF00u)) << 8);
    uint32_t e = enc & 0x1FFFFu;
    uint32_t fast = (enc >> 17) & 1u;
    uint32_t base = fast ? 12000000u : 3000000u;   // 12M/分频 或 3M/分频
    uint32_t div8;

    if (e <= 2) {
        if (fast) {
            // 编码特例: 0=12M 1=8M 2=6M
            return (e == 0) ? 12000000u : (e == 1) ? 8000000u : 6000000u;
        }
        div8 = (e + 1u) * 8u;
    } else {
        div8 = (e & 0x3FFFu) * 8u + frac_inv[(e >> 14) & 7u];
    }
    return (base * 8u + div8 / 2u) / div8;
}

static void serial_uart_apply(ser_ch_t *s)
{
    USART_InitTypeDef ui;
    uint32_t baud = s->baud;

    if (baud < 300) baud = 300;
    if (baud > 6000000) baud = 6000000;

    ui.USART_BaudRate = baud;
    ui.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    ui.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    // 数据位+校验映射到CH32的字长(8b/9b, 字长含校验位)
    if (s->bits == 7 && s->par != 0) {
        ui.USART_WordLength = USART_WordLength_8b;  // 8字长-1校验位=7数据位
    } else if (s->bits == 8 && s->par != 0) {
        ui.USART_WordLength = USART_WordLength_9b;
    } else {
        ui.USART_WordLength = USART_WordLength_8b;  // 7N没法精确, 按8N处理
    }
    ui.USART_Parity = (s->par == 1) ? USART_Parity_Odd :
                      (s->par == 2) ? USART_Parity_Even : USART_Parity_No;

    ui.USART_StopBits = (s->stop == 2) ? USART_StopBits_2 :
                        (s->stop == 1) ? USART_StopBits_1_5 : USART_StopBits_1;

    USART_Init(s->uart, &ui);
}

void ftdi_serial_init(void)
{
    GPIO_InitTypeDef gi;
    int i;

    for (i = 0; i < FT_SER_CH; i++) {
        memset(&g_ser[i], 0, sizeof(g_ser[i]));
        g_ser[i].baud = 115200;
        g_ser[i].bits = 8;
    }

    // B: USART1 PA9/PA10 (挂在APB2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC, ENABLE);
    g_ser[0].uart = USART1;
    gi.GPIO_Pin = GPIO_Pin_9;              // TTL_B TX
    gi.GPIO_Mode = GPIO_Mode_AF_PP;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gi);
    gi.GPIO_Pin = GPIO_Pin_10;             // TTL_B RX
    gi.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gi);

    // C: UART6 PC0/1, D: UART7 PC2/3, E: UART8 PC4/5 (都在APB1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6 | RCC_APB1Periph_UART7 |
                           RCC_APB1Periph_UART8, ENABLE);
    g_ser[1].uart = UART6;
    g_ser[2].uart = UART7;
    g_ser[3].uart = UART8;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4;  // TTL_C/D/E TX
    gi.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gi);
    gi.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5;  // TTL_C/D/E RX
    gi.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gi);

    for (i = 0; i < FT_SER_CH; i++) {
        serial_uart_apply(&g_ser[i]);
        USART_ITConfig(g_ser[i].uart, USART_IT_RXNE, ENABLE);
    }
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(UART6_IRQn);
    NVIC_EnableIRQ(UART7_IRQn);
    NVIC_EnableIRQ(UART8_IRQn);
}

// ---- 串口中断公共体 ----
static void serial_irq(ser_ch_t *s)
{
    if (s->uart->STATR & USART_FLAG_RXNE) {
        uint8_t d = (uint8_t)s->uart->DATAR;
        uint16_t nxt = (s->rxh + 1) & (SER_RING - 1);
        if (nxt != s->rxt) {               // 环满丢字节(跟真FTDI溢出行为差不多)
            s->rx[s->rxh] = d;
            s->rxh = nxt;
        }
    }
    if ((s->uart->STATR & USART_FLAG_TXE) && (s->uart->CTLR1 & USART_FLAG_TXE)) {
        if (s->txh != s->txt) {
            s->uart->DATAR = s->tx[s->txt];
            s->txt = (s->txt + 1) & (SER_RING - 1);
        } else {
            s->uart->CTLR1 &= ~USART_FLAG_TXE;  // 发完了关发送中断
        }
    }
}

void USART1_IRQHandler(void) { serial_irq(&g_ser[0]); }
void UART6_IRQHandler(void)  { serial_irq(&g_ser[1]); }
void UART7_IRQHandler(void)  { serial_irq(&g_ser[2]); }
void UART8_IRQHandler(void)  { serial_irq(&g_ser[3]); }

// 往tx环塞数据, 返回实际塞入数(满时部分塞入)
static uint16_t serial_tx_push(ser_ch_t *s, const uint8_t *d, uint16_t n)
{
    uint16_t done = 0;
    __disable_irq();
    while (n && (((s->txh + 1) & (SER_RING - 1)) != s->txt)) {
        s->tx[s->txh] = *d++;
        s->txh = (s->txh + 1) & (SER_RING - 1);
        n--;
        done++;
    }
    __enable_irq();
    if (done) s->uart->CTLR1 |= USART_FLAG_TXE;  // 有数据了打开发送中断
    return done;
}

// ---- 串口通道任务: OUT块→UART, UART→IN环 ----
void ftdi_serial_task(ftdi_port_t *p)
{
    ser_ch_t *s = &g_ser[p->ser_ch];

    // 1. USB OUT块 -> 串口TX环, 塞不下的留在块里(OUT端点NAK背压)
    while (p->rx_deal != p->rx_load || p->rx_stalled) {
        uint8_t blk = p->rx_deal;
        uint16_t n = p->rxlen[blk];
        uint16_t i = p->blk_pos;
        if (i < n) {
            i += serial_tx_push(s, p->rxblk[blk] + i, (uint16_t)(n - i));
            p->blk_pos = i;
        }
        if (i < n) break;  // tx环满, 等串口消化
        p->rx_deal = (uint8_t)((blk + 1) % FT_BLK_NUM);
        p->blk_pos = 0;
        if (p->rx_stalled) p->rearm_out(p);
    }

    // 2. 串口RX环 -> USB IN环(关中断领取一段, 拷贝时不关)
    {
        uint16_t k, src;
        uint16_t free_ = (uint16_t)(((p->tx_tail - p->tx_head - 1u) & (FT_TX_RING - 1)));
        __disable_irq();
        {
            uint16_t avail = (uint16_t)((s->rxh - s->rxt) & (SER_RING - 1));
            k = avail;
            if (k > free_) k = free_;
            if (k > (uint16_t)(p->maxpack - 2)) k = (uint16_t)(p->maxpack - 2);
            src = s->rxt;
            s->rxt = (uint16_t)((s->rxt + k) & (SER_RING - 1));
        }
        __enable_irq();
        while (k--) {
            p->txring[p->tx_head] = s->rx[src];
            p->tx_head = (uint16_t)((p->tx_head + 1) & (FT_TX_RING - 1));
            src = (uint16_t)((src + 1) & (SER_RING - 1));
        }
    }
}

// ---- 串口通道EP0厂商请求 ----
int ftdi_serial_ep0(ftdi_port_t *p, uint8_t req, uint16_t val, uint16_t idx,
                    uint16_t *plen, uint8_t *buf)
{
    ser_ch_t *s = &g_ser[p->ser_ch];

    switch (req) {
        case 0x00:  // SIO_RESET: 清USB缓冲+串口缓冲, 回状态包
            ftdi_port_purge(p, (uint8_t)(val & 0xFF));
            __disable_irq();
            s->rxt = s->rxh;
            s->txt = s->txh;
            __enable_irq();
            p->req |= 4;
            return 0;
        case 0x03:  // SET_BAUDRATE
            s->baud = serial_decode_baud(val, idx);
            serial_uart_apply(s);
            return 0;
        case 0x04:  // SET_DATA: 数据位/校验/停止位
            s->bits = (uint8_t)(val & 0xFF);
            s->par = (uint8_t)((val >> 8) & 0x07);
            s->stop = (uint8_t)((val >> 11) & 0x03);
            serial_uart_apply(s);
            return 0;
        case 0x01:  // MODEM_CTRL(DTR/RTS): 没接出脚, 装成功
        case 0x02:  // FLOW_CTRL: 无流控
        case 0x06:
        case 0x07:
            return 0;
        case 0x05:  // POLL_MODEM_STATUS
            buf[0] = 0x01;
            buf[1] = 0x60;
            *plen = 2;
            return 0;
        case 0x09:
            p->latency = (uint8_t)(val & 0xFF);
            return 0;
        case 0x0A:
            buf[0] = p->latency;
            *plen = 1;
            return 0;
        case 0x0B:  // SET_BITMODE: 串口通道不进MPSSE, 忽略
            return 0;
        case 0x0C:
            buf[0] = 1;  // RX脚电平没接出去, 回常高
            *plen = 1;
            return 0;
        default:
            return 0;
    }
}
