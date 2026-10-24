/********************************** (C) COPYRIGHT *******************************
* File Name    : usbhs_ftdi.c
* Description  : USBHS(PB6/PB7高速口)设备栈, FT4232H形态
*               A通道: EP1 IN + EP2 OUT, 接MPSSE引擎(JTAG)
*               B/C/D通道: EP3/4 EP5/6 EP7/8, 接三路USB TTL
*               结构参考WCH EVT的ch32v30x_usbhs_device.c
*******************************************************************************/
#include "ftdi.h"
#include "ftdi_desc.h"
#include <string.h>

#define HS_BLK_SZ   512

// USBHS端点寄存器的地址式访问(EVT例程同款基地址)
#define HS_UEP_TXLEN(n)   (*(volatile uint16_t *)(0x400234DC + (n - 1) * 4))
#define HS_UEP_TXCTRL(n)  (*(volatile uint8_t  *)(0x400234DE + (n - 1) * 4))
#define HS_UEP_RXCTRL(n)  (*(volatile uint8_t  *)(0x400234DF + (n - 1) * 4))
#define HS_UEP_TXDMA(n)   (*(volatile uint32_t *)(0x4002345C + (n - 1) * 4))
#define HS_UEP_RXDMA(n)   (*(volatile uint32_t *)(0x40023420 + (n - 1) * 4))

#define HS_NPORT    5                       // A=JTAG + B/C/D/E串口
static const uint8_t hs_epin[HS_NPORT]  = {1, 3, 5, 7, 9};
static const uint8_t hs_epout[HS_NPORT] = {2, 4, 6, 8, 10};

static __attribute__((aligned(4))) uint8_t hs_ep0[64];
static __attribute__((aligned(4))) uint8_t hs_tbuf[HS_NPORT][HS_BLK_SZ];   // EP1/3/5/7/9发送
static __attribute__((aligned(4))) uint8_t hs_outblk[HS_NPORT][FT_BLK_NUM][HS_BLK_SZ];

ftdi_port_t g_ftdi_hs;
ftdi_port_t g_ser_hs[FT_SER_CH];   // B/C/D/E = ch0..3

static uint8_t  hs_speed;             // 1=高速 0=全速
static uint8_t  hs_cfg;
static uint8_t  hs_addr;
static const uint8_t *hs_descptr;
static uint8_t  hs_reqcode, hs_reqtype;
static uint16_t hs_reqval, hs_reqidx, hs_reqlen;

void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static ftdi_port_t *hs_port_of(uint8_t epin)
{
    switch (epin) {
        case 1: return &g_ftdi_hs;
        case 3: return &g_ser_hs[0];
        case 5: return &g_ser_hs[1];
        default: return &g_ser_hs[2];
    }
}

/* ---------------- IN/OUT端点的启停(通用) ---------------- */

// 每个IN包都以2字节modem状态(01 60)开头, libftdi/FTDI驱动会自动剥掉
static void hs_kick_ep(ftdi_port_t *p, uint8_t ep, uint8_t *tbuf)
{
    uint16_t used, n, i, t;

    if (p->in_busy) return;
    used = (uint16_t)((p->tx_head - p->tx_tail) & (FT_TX_RING - 1));
    if (used == 0 && !p->want_status) return;

    tbuf[0] = 0x01;
    tbuf[1] = 0x60;

    if (used == 0) {
        // 纯状态包: 应付主机打开通道后的首次探测读, 不消耗环形缓冲
        p->want_status = 0;
        p->in_sent = 0;
        p->in_busy = 1;
        p->last_tx_tick = (uint32_t)(SysTick->CNT >> 4);
        HS_UEP_TXLEN(ep) = 2;
        HS_UEP_TXCTRL(ep) = (HS_UEP_TXCTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
        return;
    }

    n = (used > (uint16_t)(p->maxpack - 2)) ? (uint16_t)(p->maxpack - 2) : used;
    t = p->tx_tail;
    for (i = 0; i < n; i++) {
        tbuf[i + 2] = p->txring[t];
        t = (uint16_t)((t + 1) & (FT_TX_RING - 1));
    }
    p->in_sent = n;
    p->in_busy = 1;
    p->last_tx_tick = (uint32_t)(SysTick->CNT >> 4);
    HS_UEP_TXLEN(ep) = (uint16_t)(n + 2);
    HS_UEP_TXCTRL(ep) = (HS_UEP_TXCTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
}

#define MAKE_HS_KICK(name, ep, buf) \
static void name(ftdi_port_t *p) { hs_kick_ep(p, ep, buf); }

MAKE_HS_KICK(hs_kick_a, 1, hs_tbuf[0])
MAKE_HS_KICK(hs_kick_b, 3, hs_tbuf[1])
MAKE_HS_KICK(hs_kick_c, 5, hs_tbuf[2])
MAKE_HS_KICK(hs_kick_d, 7, hs_tbuf[3])
MAKE_HS_KICK(hs_kick_e, 9, hs_tbuf[4])

// OUT环腾出空位后重新放行
static void hs_rearm_ep(ftdi_port_t *p, uint8_t ep)
{
    if (!p->rx_stalled) return;
    p->rx_stalled = 0;
    HS_UEP_RXDMA(ep) = (uint32_t)p->rxblk[p->rx_load];
    HS_UEP_RXCTRL(ep) = (HS_UEP_RXCTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
}

#define MAKE_HS_REARM(name, ep) \
static void name(ftdi_port_t *p) { hs_rearm_ep(p, ep); }

MAKE_HS_REARM(hs_rearm_a, 2)
MAKE_HS_REARM(hs_rearm_b, 4)
MAKE_HS_REARM(hs_rearm_c, 6)
MAKE_HS_REARM(hs_rearm_d, 8)
MAKE_HS_REARM(hs_rearm_e, 10)

// OUT包入环(中断里调)
static void hs_out_store(ftdi_port_t *p, uint8_t ep)
{
    uint8_t blk = p->rx_load;
    uint8_t nxt;

    p->rxlen[blk] = USBHSD->RX_LEN;
    nxt = (uint8_t)((blk + 1) % FT_BLK_NUM);
    if (nxt == p->rx_deal) {
        // 环满: 先NAK住, 等主循环消化
        HS_UEP_RXCTRL(ep) = (HS_UEP_RXCTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
        p->rx_stalled = 1;
    } else {
        HS_UEP_RXDMA(ep) = (uint32_t)p->rxblk[nxt];
    }
    p->rx_load = nxt;
    HS_UEP_RXCTRL(ep) ^= USBHS_UEP_R_TOG_DATA1;
}

// IN完成(中断里调)
static void hs_in_done(ftdi_port_t *p, uint8_t ep)
{
    HS_UEP_TXCTRL(ep) = (HS_UEP_TXCTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
    HS_UEP_TXCTRL(ep) ^= USBHS_UEP_T_TOG_DATA1;
    p->in_busy = 0;
    p->tx_tail = (uint16_t)((p->tx_tail + p->in_sent) & (FT_TX_RING - 1));
}

/* ---------------- 端点/设备初始化 ---------------- */
static void hs_endp_init(void)
{
    int i;

    USBHSD->ENDP_CONFIG = USBHS_UEP1_T_EN | USBHS_UEP2_R_EN |
                          USBHS_UEP3_T_EN | USBHS_UEP4_R_EN |
                          USBHS_UEP5_T_EN | USBHS_UEP6_R_EN |
                          USBHS_UEP7_T_EN | USBHS_UEP8_R_EN |
                          USBHS_UEP9_T_EN | USBHS_UEP10_R_EN;

    USBHSD->UEP0_MAX_LEN = 64;
    USBHSD->UEP1_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP2_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP3_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP4_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP5_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP6_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP7_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP8_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP9_MAX_LEN = HS_BLK_SZ;
    USBHSD->UEP10_MAX_LEN = HS_BLK_SZ;

    USBHSD->UEP0_DMA = (uint32_t)hs_ep0;
    HS_UEP_TXDMA(1) = (uint32_t)hs_tbuf[0];
    HS_UEP_TXDMA(3) = (uint32_t)hs_tbuf[1];
    HS_UEP_TXDMA(5) = (uint32_t)hs_tbuf[2];
    HS_UEP_TXDMA(7) = (uint32_t)hs_tbuf[3];
    HS_UEP_RXDMA(2) = (uint32_t)&hs_outblk[0][0][0];
    HS_UEP_RXDMA(4) = (uint32_t)&hs_outblk[1][0][0];
    HS_UEP_RXDMA(6) = (uint32_t)&hs_outblk[2][0][0];
    HS_UEP_RXDMA(8) = (uint32_t)&hs_outblk[3][0][0];
    HS_UEP_TXDMA(9) = (uint32_t)hs_tbuf[4];
    HS_UEP_RXDMA(10) = (uint32_t)&hs_outblk[4][0][0];

    USBHSD->UEP0_TX_LEN = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
    for (i = 0; i < HS_NPORT; i++) {
        HS_UEP_TXLEN(hs_epin[i]) = 0;
        HS_UEP_TXCTRL(hs_epin[i]) = USBHS_UEP_T_RES_NAK;
        HS_UEP_RXCTRL(hs_epout[i]) = USBHS_UEP_R_RES_ACK;
    }

    g_ftdi_hs.in_busy = 0;  g_ftdi_hs.rx_stalled = 0;
    for (i = 0; i < FT_SER_CH; i++) {
        g_ser_hs[i].in_busy = 0;
        g_ser_hs[i].rx_stalled = 0;
    }
}

void usbhs_ftdi_init(void)
{
    uint8_t *blkptr[FT_BLK_NUM];
    int i, k;

    // A通道: MPSSE JTAG
    for (k = 0; k < FT_BLK_NUM; k++) blkptr[k] = hs_outblk[0][k];
    ftdi_port_init(&g_ftdi_hs, blkptr, HS_BLK_SZ, HS_BLK_SZ,
                   hs_kick_a, hs_rearm_a);

    // B/C/D: 三路TTL
    for (i = 0; i < FT_SER_CH; i++) {
        void (*kick)(ftdi_port_t *);
        void (*rearm)(ftdi_port_t *);
        switch (i) {
            case 0:  kick = hs_kick_b; rearm = hs_rearm_b; break;
            case 1:  kick = hs_kick_c; rearm = hs_rearm_c; break;
            case 2:  kick = hs_kick_d; rearm = hs_rearm_d; break;
            default: kick = hs_kick_e; rearm = hs_rearm_e; break;
        }
        for (k = 0; k < FT_BLK_NUM; k++) blkptr[k] = hs_outblk[i + 1][k];
        ftdi_port_init(&g_ser_hs[i], blkptr, HS_BLK_SZ, HS_BLK_SZ, kick, rearm);
        g_ser_hs[i].is_serial = 1;
        g_ser_hs[i].ser_ch = (uint8_t)i;
        g_ser_hs[i].bitmode = 1;   // 串口通道恒为"活动", 参与latency补包
    }

    // USBHS时钟: HSE 8M进PHY锁相环
    RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
    RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
    RCC_USBHSConfig(RCC_USBPLL_Div2);
    RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
    RCC_USBHSPHYPLLALIVEcmd(ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

    USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
    Delay_Us(10);
    USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
    USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
    USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
    // 全部中断源都要使能: 没使能的标志照样会置位, 却不会触发中断,
    // INT_BUSY机制下SIE会永久NAK(设备装死). 风暴风险靠中断里快进快出压制
    USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER |
                     USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
    hs_endp_init();
    USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
    NVIC_EnableIRQ(USBHS_IRQn);
}

/* ---------------- EP0 IN后续阶段 ---------------- */
static void hs_ep0_in_stage(void)
{
    if (hs_reqlen == 0) {
        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    }
    if ((hs_reqtype & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
        if (hs_reqcode == USB_GET_DESCRIPTOR) {
            uint16_t len = hs_reqlen >= 64 ? 64 : hs_reqlen;
            memcpy(hs_ep0, hs_descptr, len);
            hs_reqlen -= len;
            hs_descptr += len;
            USBHSD->UEP0_TX_LEN = len;
            USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
            return;
        }
        if (hs_reqcode == USB_SET_ADDRESS) {
            USBHSD->DEV_AD = hs_addr;
        }
    }
    USBHSD->UEP0_TX_LEN = 0;
}

// 端点STALL的置位/清除(SET/CLEAR_FEATURE用)
static void hs_ep_stall(uint8_t ep, uint8_t stall)
{
    if (ep & 0x80) {
        uint8_t n = ep & 0x0F;
        HS_UEP_TXCTRL(n) = (HS_UEP_TXCTRL(n) & ~USBHS_UEP_T_RES_MASK) |
                (stall ? USBHS_UEP_T_RES_STALL : USBHS_UEP_T_RES_NAK);
    } else {
        HS_UEP_RXCTRL(ep) = (HS_UEP_RXCTRL(ep) & ~USBHS_UEP_R_RES_MASK) |
                (stall ? USBHS_UEP_R_RES_STALL : USBHS_UEP_R_RES_ACK);
    }
}

/* ---------------- 中断 ---------------- */
void USBHS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;
    PUSB_SETUP_REQ preq = (PUSB_SETUP_REQ)hs_ep0;

    intflag = USBHSD->INT_FG;
    intst = USBHSD->INT_ST;

    // 注意: 多个标志可能同时置位(比如BUS_RST撞上传输完成),
    // 必须挨个处理再统一清除, 否则被吞掉的那个会让状态机失步
    if (intflag & USBHS_UIF_TRANSFER) {
        switch (intst & USBHS_UIS_TOKEN_MASK) {
            case USBHS_UIS_TOKEN_IN:
                switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {
                    case USBHS_UIS_TOKEN_IN | 0:
                        hs_ep0_in_stage();
                        break;

                    case USBHS_UIS_TOKEN_IN | 1:
                        hs_in_done(&g_ftdi_hs, 1);
                        break;
                    case USBHS_UIS_TOKEN_IN | 3:
                        hs_in_done(&g_ser_hs[0], 3);
                        break;
                    case USBHS_UIS_TOKEN_IN | 5:
                        hs_in_done(&g_ser_hs[1], 5);
                        break;
                    case USBHS_UIS_TOKEN_IN | 7:
                        hs_in_done(&g_ser_hs[2], 7);
                        break;
                    case USBHS_UIS_TOKEN_IN | 9:
                        hs_in_done(&g_ser_hs[3], 9);
                        break;

                    default:
                        break;
                }
                break;

            case USBHS_UIS_TOKEN_OUT:
                switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {
                    case USBHS_UIS_TOKEN_OUT | 0:
                        if (intst & USBHS_UIS_TOG_OK) {
                            if (hs_reqlen == 0) {
                                USBHSD->UEP0_TX_LEN = 0;
                                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                            }
                        }
                        break;

                    case USBHS_UIS_TOKEN_OUT | 2:
                        hs_out_store(&g_ftdi_hs, 2);
                        break;
                    case USBHS_UIS_TOKEN_OUT | 4:
                        hs_out_store(&g_ser_hs[0], 4);
                        break;
                    case USBHS_UIS_TOKEN_OUT | 6:
                        hs_out_store(&g_ser_hs[1], 6);
                        break;
                    case USBHS_UIS_TOKEN_OUT | 8:
                        hs_out_store(&g_ser_hs[2], 8);
                        break;
                    case USBHS_UIS_TOKEN_OUT | 10:
                        hs_out_store(&g_ser_hs[3], 10);
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
    if (intflag & USBHS_UIF_SETUP_ACT) {
        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

        hs_reqtype = preq->bRequestType;
        hs_reqcode = preq->bRequest;
        hs_reqlen  = preq->wLength;
        hs_reqval  = preq->wValue;
        hs_reqidx  = preq->wIndex;
        len = 0;
        errflag = 0;

        if ((hs_reqtype & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
            // 厂商/类请求: 按wIndex低字节路由到A通道(MPSSE)或B/C/D(TTL)
            if (hs_reqtype & USB_REQ_TYP_VENDOR) {
                uint8_t ch = (uint8_t)(hs_reqidx & 0xFF);
                int r;
                if (ch == 0 || ch == 1) {
                    r = ftdi_ep0_vendor(&g_ftdi_hs, hs_reqcode, hs_reqval,
                                        hs_reqidx, &hs_reqlen, hs_ep0);
                } else if (ch >= 2 && ch <= 1 + FT_SER_CH) {
                    r = ftdi_serial_ep0(&g_ser_hs[ch - 2], hs_reqcode, hs_reqval,
                                        hs_reqidx, &hs_reqlen, hs_ep0);
                } else {
                    r = 0xFF;
                }
                errflag = (uint8_t)r;
                if (errflag != 0xFF) {
                    len = hs_reqlen;   // 厂商IN响应已放进hs_ep0
                }
            } else {
                errflag = 0xFF;  // 类请求不支持
            }
        } else {
            switch (hs_reqcode) {
                case USB_GET_DESCRIPTOR: {
                    // 先刷新当前速率(影响配置描述符选择)
                    hs_speed = ((USBHSD->SPEED_TYPE & 0x03) == 0x01) ? 1 : 0;
                    if (ftdi_get_descriptor(hs_speed, hs_reqval, &hs_descptr, &len) != 0) {
                        errflag = 0xFF;
                    } else {
                        if (hs_reqlen > len) hs_reqlen = len;
                        len = (hs_reqlen >= 64) ? 64 : hs_reqlen;
                        memcpy(hs_ep0, hs_descptr, len);
                        hs_descptr += len;
                    }
                    break;
                }

                case USB_SET_ADDRESS:
                    hs_addr = (uint8_t)(hs_reqval & 0xFF);
                    break;

                case USB_GET_CONFIGURATION:
                    hs_ep0[0] = hs_cfg;
                    if (hs_reqlen > 1) hs_reqlen = 1;
                    len = hs_reqlen;
                    break;

                case USB_SET_CONFIGURATION:
                    hs_cfg = (uint8_t)(hs_reqval & 0xFF);
                    break;

                case USB_GET_INTERFACE:
                    hs_ep0[0] = 0;
                    if (hs_reqlen > 1) hs_reqlen = 1;
                    len = hs_reqlen;
                    break;

                case USB_SET_INTERFACE:
                    break;

                case USB_GET_STATUS:
                    hs_ep0[0] = 0;
                    hs_ep0[1] = 0;
                    if (hs_reqlen > 2) hs_reqlen = 2;
                    len = hs_reqlen;
                    break;

                case USB_CLEAR_FEATURE:
                case USB_SET_FEATURE:
                    if ((hs_reqtype & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        uint8_t ep = (uint8_t)(hs_reqidx & 0xFF);
                        if (hs_reqval != 0) { errflag = 0xFF; break; }
                        switch (ep) {
                            case 0x81: case 0x01:
                            case 0x83: case 0x03:
                            case 0x85: case 0x05:
                            case 0x87: case 0x07:
                            case 0x89: case 0x09:
                            case 0x02: case 0x04: case 0x06: case 0x08: case 0x0A:
                                hs_ep_stall(ep, (hs_reqcode == USB_SET_FEATURE));
                                break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                    } else if ((hs_reqtype & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        // 远程唤醒之类, 装允许
                    } else {
                        errflag = 0xFF;
                    }
                    break;

                default:
                    errflag = 0xFF;
                    break;
            }
        }

        if (errflag == 0xFF) {
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
        } else {
            if (hs_reqtype & 0x80) {
                // IN方向: 第一包数据已在setup阶段放好
                USBHSD->UEP0_TX_LEN = len;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                if ((hs_reqtype & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD &&
                    hs_reqcode == USB_GET_DESCRIPTOR) {
                    hs_reqlen -= len;  // 描述符续传
                } else {
                    hs_reqlen = 0;     // 厂商响应一发完事
                }
            } else {
                if (hs_reqlen == 0) {
                    USBHSD->UEP0_TX_LEN = 0;
                    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                } else {
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }
            }
        }
    }
    if (intflag & USBHS_UIF_BUS_RST) {
        int i;
        hs_cfg = 0;
        hs_addr = 0;
        USBHSD->DEV_AD = 0;
        hs_endp_init();
        ftdi_port_reset(&g_ftdi_hs);
        for (i = 0; i < FT_SER_CH; i++) ftdi_port_reset(&g_ser_hs[i]);
    }
    USBHSD->INT_FG = intflag;  // 统一清掉本次处理过的标志
}
