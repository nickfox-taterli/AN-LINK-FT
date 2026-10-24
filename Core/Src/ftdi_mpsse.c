/********************************** (C) COPYRIGHT *******************************
* File Name    : ftdi_mpsse.c
* Description  : FTDI MPSSE命令引擎 + JTAG位操作
*               语义参考libftdi/ch55x_jtag/FTDI AN108, 尽量做全命令兼容:
*               - 0x00~0x3F 数据移位族(只读/只写/读写, 字节/位, LSB/MSB, 正负沿全组合)
*                 注意只读命令后面没有数据字节, 设备自己空转打1读回
*               - 0x40~0x4F/0x60~0x6F TMS族(可带TDO读)
*               - 0x80~0x83 GPIO低/高字节读写
*               - 0x84/0x85 环回, 0x86 TCK分频, 0x87 立即回传
*               - 0x88/0x89 等待ADBUS0, 0x8A/0x8B 5分频, 0x8C/0x8D 三相时钟
*               - 0x8E/0x8F 空转时钟(2232H, openFPGALoader的toggleClk会发)
*               - 0x90以后(自适应时钟等): 收下装成功
*               - 非法命令: 按FTDI协议回 0xFA + 命令字节
*******************************************************************************/
#include "ftdi.h"
#include "ftdi_desc.h"
#include "ftdi_spi.h"
#include <string.h>

// ---- 内联延时: 每个滴答约3个CPU周期(addi+nop+bne) ----
static inline void ft_dly(uint32_t n)
{
    while (n--) {
        __asm volatile("nop");
    }
}

// 活动指示, main里点灯用
volatile uint8_t g_ftdi_active;

// 毫秒级时基: SysTick自由计数(HCLK/8), 右移4位后约1.1MHz, 用32位差值防溢出
#define FT_TICKS_PER_MS  ((SystemCoreClock / 8 / 16 / 1000) + 1)
static inline uint32_t ft_tick(void)
{
    return (uint32_t)(SysTick->CNT >> 4);
}

void ftdi_jtag_gpio_init(void)
{
    GPIO_InitTypeDef gi;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // TCK/TMS/TDI 推挽输出, 50MHz摆率
    gi.GPIO_Pin = FT_PIN_TCK | FT_PIN_TMS | FT_PIN_TDI;
    gi.GPIO_Mode = GPIO_Mode_Out_PP;
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gi);

    // TDO 上拉输入(不接FPGA时电平也稳定)
    gi.GPIO_Pin = FT_PIN_TDO;
    gi.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gi);

    // 初始状态: TCK=0, TMS=1(测试逻辑复位方向), TDI=1
    GPIOB->BSHR = FT_TCK_LO | FT_TMS_SET | FT_TDI_SET;
}

// 读ADBUS0..3实际电平: bit0=TCK bit1=TDI bit2=TDO bit3=TMS
uint8_t ftdi_read_adbus(void)
{
    uint32_t idr = GPIOB->INDR;
    uint8_t v = 0;
    if (idr & FT_PIN_TCK) v |= 0x01;
    if (idr & FT_PIN_TDI) v |= 0x02;
    if (idr & FT_PIN_TDO) v |= 0x04;
    if (idr & FT_PIN_TMS) v |= 0x08;
    return v;
}

// SET_BITS_LOW: 按FTDI线序把val/dir落到物理脚, dir位=1输出
void ftdi_set_adbus(uint8_t val, uint8_t dir)
{
    GPIO_InitTypeDef gi;

    // 先把输出位按val设好, 再切方向
    GPIOB->BSHR = ((val & 0x01) ? FT_TCK_HI : FT_TCK_LO) |
                  ((val & 0x02) ? FT_TDI_SET : FT_TDI_CLR) |
                  ((val & 0x08) ? FT_TMS_SET : FT_TMS_CLR);

    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Pin = FT_PIN_TCK;
    gi.GPIO_Mode = (dir & 0x01) ? GPIO_Mode_Out_PP : GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gi);
    gi.GPIO_Pin = FT_PIN_TDI;
    gi.GPIO_Mode = (dir & 0x02) ? GPIO_Mode_Out_PP : GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gi);
    gi.GPIO_Pin = FT_PIN_TDO;
    gi.GPIO_Mode = (dir & 0x04) ? GPIO_Mode_Out_PP : GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gi);
    gi.GPIO_Pin = FT_PIN_TMS;
    gi.GPIO_Mode = (dir & 0x08) ? GPIO_Mode_Out_PP : GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gi);
}

/* ---------------- 各种位泵 ---------------- */

// 热路径: 字节读写, LSB先, 负沿写/正沿读(0x39/0x19/0x29/0x21等)
static inline uint8_t jtag_io_lsb(uint8_t d, int rd, uint32_t dly)
{
    uint8_t r = 0;
    int i;
    for (i = 0; i < 8; i++) {
        GPIOB->BSHR = FT_TCK_LO | ((d & 1) ? FT_TDI_SET : FT_TDI_CLR);  // TCK负沿+TDI有效
        if (dly) ft_dly(dly);
        r >>= 1;
        if (rd && (GPIOB->INDR & FT_TDO_MSK)) r |= 0x80;                  // 上沿前采样
        GPIOB->BSHR = FT_TCK_HI;
        if (dly) ft_dly(dly);
        d >>= 1;
    }
    return r;
}

// 热路径: 字节读写, MSB先
static inline uint8_t jtag_io_msb(uint8_t d, int rd, uint32_t dly)
{
    uint8_t r = 0;
    int i;
    for (i = 0; i < 8; i++) {
        GPIOB->BSHR = FT_TCK_LO | ((d & 0x80) ? FT_TDI_SET : FT_TDI_CLR);
        if (dly) ft_dly(dly);
        r <<= 1;
        if (rd && (GPIOB->INDR & FT_TDO_MSK)) r |= 1;
        GPIOB->BSHR = FT_TCK_HI;
        if (dly) ft_dly(dly);
        d <<= 1;
    }
    return r;
}

// 万能位泵: 任意边沿组合/三位相时钟走这里, 慢一点但全兼容
static inline uint8_t jtag_bit_gen(int tdi, int rd, int wneg, int rneg,
                                   int phase3, uint32_t dly)
{
    uint32_t tm = tdi ? FT_TDI_SET : FT_TDI_CLR;
    uint8_t s = 0;

    if (wneg) {
        GPIOB->BSHR = FT_TCK_LO | tm;                 // 负沿打数据
        if (dly) ft_dly(dly); else (void)GPIOB->INDR;
        if (rd && rneg) s = (GPIOB->INDR & FT_TDO_MSK) ? 1 : 0;
        GPIOB->BSHR = FT_TCK_HI;
        if (dly) ft_dly(dly);
        if (rd && !rneg) s = (GPIOB->INDR & FT_TDO_MSK) ? 1 : 0;
        if (phase3) {                                 // 三相: 再补一个负沿
            GPIOB->BSHR = FT_TCK_LO;
            if (dly) ft_dly(dly);
        }
    } else {
        GPIOB->BSHR = tm;                             // 数据先变, TCK保持低
        if (dly) ft_dly(dly); else (void)GPIOB->INDR;
        GPIOB->BSHR = FT_TCK_HI;                      // 正沿打数据
        if (dly) ft_dly(dly); else if (!rd || rneg) (void)GPIOB->INDR;
        if (rd && !rneg) s = (GPIOB->INDR & FT_TDO_MSK) ? 1 : 0;
        GPIOB->BSHR = FT_TCK_LO;
        if (rd && rneg) { if (dly) ft_dly(dly); s = (GPIOB->INDR & FT_TDO_MSK) ? 1 : 0; }
        else if (dly) ft_dly(dly);
    }
    return s;
}

// TMS序列: data的bit0..6是TMS序列, bit7是常驻TDI
// n=时钟个数-1; 采样排列跟ch55x_jtag/openFPGALoader约定一致:
// 第k次采样落在bit(8-n+k-1), 也就是最后一次采样在bit7
static uint8_t jtag_tms(uint8_t data, uint8_t n, int rd, int wneg, int rneg,
                        uint32_t dly)
{
    uint32_t tdi = (data & 0x80) ? FT_TDI_SET : FT_TDI_CLR;
    uint8_t r = 0;
    int k;
    for (k = 0; k <= n; k++) {
        uint32_t tms = ((data >> k) & 1) ? FT_TMS_SET : FT_TMS_CLR;
        if (wneg) {
            GPIOB->BSHR = FT_TCK_LO | tdi | tms;
            if (dly) ft_dly(dly); else if (!rd || !rneg) (void)GPIOB->INDR;
            r >>= 1;
            if (rd && rneg && (GPIOB->INDR & FT_TDO_MSK)) r |= 0x80;
            GPIOB->BSHR = FT_TCK_HI;
            if (dly) ft_dly(dly);
            if (rd && !rneg && (GPIOB->INDR & FT_TDO_MSK)) r |= 0x80;
        } else {
            GPIOB->BSHR = tdi | tms;                  // 数据先变
            if (dly) ft_dly(dly);
            GPIOB->BSHR = FT_TCK_HI;
            if (dly) ft_dly(dly);
            r >>= 1;
            if (rd && !rneg && (GPIOB->INDR & FT_TDO_MSK)) r |= 0x80;
            GPIOB->BSHR = FT_TCK_LO;
            if (rd && rneg) { if (dly) ft_dly(dly); if (GPIOB->INDR & FT_TDO_MSK) r |= 0x80; }
            else if (dly) ft_dly(dly);
        }
    }
    return r;
}

// 空转时钟: 输出电平保持, 只打n个TCK周期
static void jtag_run_cycles(uint32_t n, uint32_t dly)
{
    while (n--) {
        GPIOB->BSHR = FT_TCK_LO;
        if (dly) ft_dly(dly); else (void)GPIOB->INDR;  // 读屏障: 防相邻双写被存储缓冲合并
        GPIOB->BSHR = FT_TCK_HI;
        if (dly) ft_dly(dly);
    }
}

/* ---------------- IN环形缓冲 ---------------- */
static const uint8_t ff_pattern[FTDI_SPI_BOUNCE_SZ] = {[0 ... FTDI_SPI_BOUNCE_SZ - 1] = 0xFF};  // 只读命令打1用
static void mpsse_shift_block(ftdi_port_t *p, const uint8_t *src, uint16_t n);

static void in_put(ftdi_port_t *p, uint8_t b)
{
    uint16_t nxt = (uint16_t)((p->tx_head + 1) & (FT_TX_RING - 1));
    if (nxt == p->tx_tail) {
        return;  // 环满丢字节, 协议正常时不会发生
    }
    p->txring[p->tx_head] = b;
    p->tx_head = nxt;
    {
        uint16_t used = (uint16_t)((p->tx_head - p->tx_tail) & (FT_TX_RING - 1));
        if (used >= p->maxpack) p->kick_in(p);  // 攒够整包就试着发
    }
}

/* ---------------- 时速换算 ---------------- */
static void ft_apply_divisor(ftdi_port_t *p)
{
    // This firmware is a fixed-speed TD/FD cable. Divisor commands are
    // accepted for compatibility; bulk transfers always use SPI2 + DMA.
    p->spi_ok = 1;
    p->dly = 0;
}

/* ---------------- 移位执行 ---------------- */

// 移一个字节, 命令语义在p->cmd里; 只读时d无效(打1)
static uint8_t shift_one_byte(ftdi_port_t *p, uint8_t d)
{
    uint8_t c = p->cmd;
    int rd = c & 0x20;
    uint8_t r;

    if (!p->phase3 && (c & 0x01) && !(c & 0x04)) {
        // 最常见组合(负沿写正沿读)走快速通道
        r = (c & 0x08) ? jtag_io_lsb(d, rd, p->dly) : jtag_io_msb(d, rd, p->dly);
    } else {
        int i;
        uint8_t rr = 0;
        if (c & 0x08) {
            for (i = 0; i < 8; i++, d >>= 1)
                rr |= jtag_bit_gen(d & 1, rd, c & 0x01, c & 0x04, p->phase3, p->dly) << i;
        } else {
            for (i = 7; i >= 0; i--, d <<= 1)
                rr |= jtag_bit_gen((d >> 7) & 1, rd, c & 0x01, c & 0x04, p->phase3, p->dly) << i;
        }
        r = rr;
    }
    return r;
}

// 移n位(1~8), 数据在d
static uint8_t shift_bits(ftdi_port_t *p, uint8_t d, uint8_t n)
{
    uint8_t c = p->cmd;
    int rd = c & 0x20;
    uint8_t r = 0;
    int i;

    if (c & 0x08) {
        for (i = 0; i < n; i++, d >>= 1)
            r |= jtag_bit_gen(d & 1, rd, c & 0x01, c & 0x04, p->phase3, p->dly) << i;
    } else {
        for (i = n - 1; i >= 0; i--, d <<= 1)
            r |= jtag_bit_gen((d >> 7) & 1, rd, c & 0x01, c & 0x04, p->phase3, p->dly) << i;
    }
    return (uint8_t)(r << (8 - n));
}

// 只读命令: 主机不发数据, 设备空转计数个字节/位, TDI打1
static void shift_readonly(ftdi_port_t *p)
{
    uint8_t c = p->cmd;

    if (c & 0x02) {
        // 位模式, 1~8个时钟
        uint8_t r = shift_bits(p, 0xFF, (uint8_t)p->cnt);
        if (p->loopback) r = (uint8_t)(0xFFu << (8 - p->cnt));
        in_put(p, r);
    } else {
        // 字节模式: 分块走批量路径(SPI硬件或位操作), IN环满时分段
        uint16_t n = p->cnt;
        while (n) {
            uint16_t k = n > 512 ? 512 : n;
            mpsse_shift_block(p, ff_pattern, k);
            n -= k;
        }
    }
}

/* ---------------- 命令流状态机 ---------------- */
static void mpsse_dispatch(ftdi_port_t *p, uint8_t c)
{
    if (c >= 0x40) {
        if (c & 0x40 && (c & 0xE0) != 0x40 && (c & 0xE0) != 0x60) {
            // 0x50~0x5F/0x70~0x7F保留段, 当非法处理
            in_put(p, 0xFA);
            p->st = MS_ERR_ECHO;
            return;
        }
        if (c >= 0x80) {
            // 特殊命令: 全部要GPIO引脚(时钟/GPIO操作/读引脚)
            ftdi_spi_release();
            switch (c) {
                case 0x80: case 0x82:
                    p->cmd = c;
                    p->st = MS_GPIO_VAL;
                    return;
                case 0x81:
                    in_put(p, ftdi_read_adbus());
                    return;
                case 0x83:
                    in_put(p, p->high_val);
                    return;
                case 0x84: p->loopback = 1; return;
                case 0x85: p->loopback = 0; return;
                case 0x86: p->st = MS_DIV_LO; return;
                case 0x87:
                    p->kick_in(p);  // 立即回传
                    return;
                case 0x88: {  // 等ADBUS0(TCK脚)变高, 有界等待防卡死
                    uint32_t t = 3000000;
                    while (t-- && !(GPIOB->INDR & FT_PIN_TCK));
                    return;
                }
                case 0x89: {
                    uint32_t t = 3000000;
                    while (t-- && (GPIOB->INDR & FT_PIN_TCK));
                    return;
                }
                case 0x8A: p->div5 = 1; ft_apply_divisor(p); return;
                case 0x8B: p->div5 = 0; ft_apply_divisor(p); return;
                case 0x8C: p->phase3 = 1; return;
                case 0x8D: p->phase3 = 0; return;
                case 0x8E: p->st = MS_CKB_LEN; return;
                case 0x8F: p->st = MS_CKBYTES_LO; return;
                default:
                    // 0x90~0x9D自适应时钟等, 收下装成功
                    return;
            }
        }
        // 0x40~0x7F数据段外: TMS族, 需要GPIO引脚
        ftdi_spi_release();
        p->cmd = c;
        p->st = MS_TMS_LEN;
        return;
    }

    // 0x00~0x3F: 数据移位族, 必须带读或写
    if ((c & 0x30) == 0) {
        in_put(p, 0xFA);
        p->st = MS_ERR_ECHO;
        return;
    }
    // 位模式命令走位操作, 需要引脚在GPIO模式(SPI留给字节模式批处理)
    if (c & 0x02) ftdi_spi_release();
    p->cmd = c;
    p->st = MS_LEN_LO;
}

static void mpsse_feed(ftdi_port_t *p, uint8_t b)
{
    switch (p->st) {
        case MS_IDLE:
            mpsse_dispatch(p, b);
            break;

        case MS_ERR_ECHO:
            in_put(p, b);  // 非法命令的回显字节
            p->st = MS_IDLE;
            break;

        case MS_LEN_LO:
            if (p->cmd & 0x02) {
                // 位模式: 长度=低3位+1
                p->cnt = (uint32_t)((b & 0x07) + 1);
                if ((p->cmd & 0x30) == 0x20) {
                    // 只读: 后面没有数据字节, 直接空转读回
                    uint8_t r = shift_bits(p, 0xFF, (uint8_t)p->cnt);
                    if (p->loopback) r = (uint8_t)(0xFFu << (8 - p->cnt));
                    in_put(p, r);
                    p->st = MS_IDLE;
                } else {
                    // 带写: 等一个数据字节
                    p->st = MS_DATA;
                }
            } else {
                p->cnt = b;
                p->st = MS_LEN_HI;
            }
            break;

        case MS_LEN_HI:
            p->cnt |= (uint32_t)b << 8;
            p->cnt += 1;   // 长度从0计数, 实际个数+1(0xFFFF+1=65536要32位)
            if ((p->cmd & 0x30) == 0x20) {
                shift_readonly(p);  // 只读: 后面没有数据字节
                p->st = MS_IDLE;
            } else {
                p->st = MS_DATA;
            }
            break;

        case MS_DATA:
            if (p->cmd & 0x02) {
                // 位模式: 一个数据字节搞定
                uint8_t n = (uint8_t)p->cnt;
                uint8_t r = shift_bits(p, b, n);
                if (p->loopback)
                    r = (p->cmd & 0x08) ? (uint8_t)(b & ((1u << n) - 1u))
                                        : (uint8_t)(b >> (8 - n));
                if (p->loopback) r = (uint8_t)(r << (8 - n));
                if (p->cmd & 0x20) in_put(p, r);
                p->st = MS_IDLE;
            } else {
                uint8_t r = shift_one_byte(p, b);
                if (p->loopback) r = b;
                if (p->cmd & 0x20) in_put(p, r);
                if (--p->cnt == 0) p->st = MS_IDLE;
            }
            break;

        case MS_TMS_LEN:
            p->cnt = b;   // 位数-1, 最多7位(0x07)
            p->st = MS_TMS_DAT;
            break;

        case MS_TMS_DAT: {
            uint8_t c = p->cmd;
            uint8_t r = jtag_tms(b, (uint8_t)(p->cnt & 0x07),
                                 (c & 0x20) ? 1 : 0, c & 0x01, c & 0x04, p->dly);
            if (p->loopback) r = b;
            if (c & 0x20) in_put(p, r);
            p->st = MS_IDLE;
            break;
        }

        case MS_GPIO_VAL:
            p->gpio_stage = b;
            p->st = MS_GPIO_DIR;
            break;

        case MS_GPIO_DIR:
            if (p->cmd == 0x80) {
                p->low_val = p->gpio_stage;
                p->low_dir = b;
                ftdi_set_adbus(p->gpio_stage, b);
            } else {
                p->high_val = p->gpio_stage;  // 高字节没接东西, 存着
                p->high_dir = b;
            }
            p->st = MS_IDLE;
            break;

        case MS_DIV_LO:
            p->divisor = b;
            p->st = MS_DIV_HI;
            break;

        case MS_DIV_HI:
            p->divisor |= (uint16_t)b << 8;
            ft_apply_divisor(p);
            p->st = MS_IDLE;
            break;

        case MS_CKB_LEN:  // 0x8E: n+1个空转周期
            jtag_run_cycles((uint32_t)b + 1, p->dly);
            p->st = MS_IDLE;
            break;

        case MS_CKBYTES_LO:
            p->cnt = b;
            p->st = MS_CKBYTES_HI;
            break;

        case MS_CKBYTES_HI:
            p->cnt |= (uint32_t)b << 8;
            jtag_run_cycles(((uint32_t)p->cnt + 1) * 8, p->dly);
            p->st = MS_IDLE;
            break;

        default:
            p->st = MS_IDLE;
            break;
    }
}

/* ---------------- 批量移位(热路径) ----------------
 * 字节模式带写的数据流整段移位, 结果直接写进IN环,
 * 不再逐字节过状态机, 省掉每字节的分发/调用开销
 */
static void mpsse_shift_block(ftdi_port_t *p, const uint8_t *src, uint16_t n)
{
    uint8_t c = p->cmd;
    uint16_t head = p->tx_head;
    int bit;

    // Fixed 18MHz bulk path: mode-0 byte shifts go through SPI2 + DMA.
    // Host divisor commands do not throttle this dedicated TD/FD cable.
    if (p->spi_ok && !p->phase3 && !p->loopback && !(c & 0x04) &&
        ((c & 0x10) == 0 || (c & 0x01) != 0)) {
        int wr = c & 0x10;
        int rd = c & 0x20;
        ftdi_spi_engage(c & 0x08);
        while (n) {
            uint16_t k = n > FTDI_SPI_BOUNCE_SZ ? FTDI_SPI_BOUNCE_SZ : n;
            if (rd) {
                // 结果要进IN环, 受剩余空间限制
                uint16_t fr = (uint16_t)(((p->tx_tail - head - 1u) & (FT_TX_RING - 1)));
                if (k > fr) k = fr;
                if (k == 0) break;   // 环满, 等USB中断消化
            }
            if (ftdi_spi_shift(wr ? src : ff_pattern, k) != 0) break;  // 超时兜底
            if (rd) {
                const uint8_t *b = ftdi_spi_bounce();
                uint16_t remaining = k;
                while (remaining--) {
                    p->txring[head] = *b++;
                    head = (uint16_t)((head + 1) & (FT_TX_RING - 1));
                }
            }
            src += k;
            n -= k;
        }
        p->tx_head = head;
        {
            uint16_t used = (uint16_t)((p->tx_head - p->tx_tail) & (FT_TX_RING - 1));
            if (used >= p->maxpack) p->kick_in(p);
        }
        return;
    }

    // 位操作路径: 先确保引脚在GPIO模式(TMS等命令需要)
    ftdi_spi_release();

    if (!p->phase3 && (c & 0x01) && !(c & 0x04)) {
        // 最常见: 负沿写正沿读, 全速, 按读/写和位序分四个专用循环
        if (c & 0x08) {
            if (c & 0x20) {         // LSB 读写 (0x39)
                while (n--) {
                    uint8_t d = *src++;
                    uint8_t r = 0;
                    for (bit = 0; bit < 8; bit++) {
                        GPIOB->BSHR = FT_TCK_LO | ((d & 1) ? FT_TDI_SET : FT_TDI_CLR);
                        if (p->dly) ft_dly(p->dly);
                        r >>= 1;
                        if (GPIOB->INDR & FT_TDO_MSK) r |= 0x80;
                        GPIOB->BSHR = FT_TCK_HI;
                        if (p->dly) ft_dly(p->dly);
                        d >>= 1;
                    }
                    p->txring[head] = r;
                    head = (uint16_t)((head + 1) & (FT_TX_RING - 1));
                }
            } else {                // LSB 只写 (0x19/0x11等)
                while (n--) {
                    uint8_t d = *src++;
                    for (bit = 0; bit < 8; bit++) {
                        GPIOB->BSHR = FT_TCK_LO | ((d & 1) ? FT_TDI_SET : FT_TDI_CLR);
                        if (p->dly) ft_dly(p->dly); else (void)GPIOB->INDR;
                        GPIOB->BSHR = FT_TCK_HI;
                        if (p->dly) ft_dly(p->dly);
                        d >>= 1;
                    }
                }
            }
        } else {
            if (c & 0x20) {         // MSB 读写 (0x31)
                while (n--) {
                    uint8_t d = *src++;
                    uint8_t r = 0;
                    for (bit = 0; bit < 8; bit++) {
                        GPIOB->BSHR = FT_TCK_LO | ((d & 0x80) ? FT_TDI_SET : FT_TDI_CLR);
                        if (p->dly) ft_dly(p->dly);
                        r <<= 1;
                        if (GPIOB->INDR & FT_TDO_MSK) r |= 1;
                        GPIOB->BSHR = FT_TCK_HI;
                        if (p->dly) ft_dly(p->dly);
                        d <<= 1;
                    }
                    p->txring[head] = r;
                    head = (uint16_t)((head + 1) & (FT_TX_RING - 1));
                }
            } else {                // MSB 只写
                while (n--) {
                    uint8_t d = *src++;
                    for (bit = 0; bit < 8; bit++) {
                        GPIOB->BSHR = FT_TCK_LO | ((d & 0x80) ? FT_TDI_SET : FT_TDI_CLR);
                        if (p->dly) ft_dly(p->dly); else (void)GPIOB->INDR;
                        GPIOB->BSHR = FT_TCK_HI;
                        if (p->dly) ft_dly(p->dly);
                        d <<= 1;
                    }
                }
            }
        }
    } else {
        // 罕见组合(三相/正沿/带延时): 退回逐字节万能泵
        while (n--) {
            uint8_t r = shift_one_byte(p, *src++);
            if (c & 0x20) {
                p->txring[head] = r;
                head = (uint16_t)((head + 1) & (FT_TX_RING - 1));
            }
        }
    }
    p->tx_head = head;

    // 攒够一个整包就立即冲, 缩短回传延迟
    {
        uint16_t used = (uint16_t)((p->tx_head - p->tx_tail) & (FT_TX_RING - 1));
        if (used >= p->maxpack) p->kick_in(p);
    }
}

/* ---------------- 端口生命周期 ---------------- */
void ftdi_port_init(ftdi_port_t *p, uint8_t **blocks, uint16_t blksz,
                    uint16_t maxpack,
                    void (*kick)(ftdi_port_t *), void (*rearm)(ftdi_port_t *))
{
    int i;
    memset(p, 0, sizeof(*p));
    for (i = 0; i < FT_BLK_NUM; i++) p->rxblk[i] = blocks[i];
    p->rxblksz = blksz;
    p->maxpack = maxpack;
    p->kick_in = kick;
    p->rearm_out = rearm;
    p->latency = 16;   // FTDI上电默认16ms
    p->div5 = 1;       // 2232系列上电默认5分频开, MPSSE初始化会发0x8B关
    p->divisor = 0;
    ft_apply_divisor(p);
}

void ftdi_port_reset(ftdi_port_t *p)
{
    // 从USB中断里调, 只挂请求, 主循环应用
    p->req |= 2;
}

void ftdi_port_purge(ftdi_port_t *p, uint8_t which)
{
    // 从EP0(USB中断)里调, 只挂请求, 主循环应用
    (void)which;
    p->req |= 1;
}

void ftdi_port_task(ftdi_port_t *p)
{
    // 先应用挂起的请求
    uint8_t rq = p->req;
    if (rq) {
        p->req = 0;
        p->st = MS_IDLE;
        p->rx_deal = p->rx_load;
        p->blk_pos = 0;
        p->tx_tail = p->tx_head;
        p->in_busy = 0;
        p->rx_stalled = 0;
        if (rq & 2) p->loopback = 0;
        if (rq & 4) p->want_status = 1;  // 进MPSSE: 补发一个状态包
    }

    if (p->is_serial) {
        // 串口通道: OUT块搬运到UART, UART数据搬进IN环
        ftdi_serial_task(p);
    } else {
        // MPSSE通道: 逐块消化, 字节流命令走批量快速路径
        // The USB ISR advances rx_load even when filling the last slot.
        // Equal indices plus rx_stalled means full, not empty.
        while (p->rx_deal != p->rx_load || p->rx_stalled) {
            uint8_t blk = p->rx_deal;
            uint16_t n = p->rxlen[blk];
            uint16_t i = p->blk_pos;
            uint8_t *d = p->rxblk[blk];

            g_ftdi_active = 20;  // 活动指示衰减计数

            while (i < n) {
                // 字节模式带写的数据流: 一口气移掉一整段, 不逐字节走状态机
                // Bit commands also enter MS_DATA, but consume exactly one
                // payload byte. Treating their bit count as a byte count eats
                // following MPSSE opcodes and corrupts the TAP state.
                if (p->st == MS_DATA && !p->loopback &&
                    !(p->cmd & 0x02) && (p->cmd & 0x10)) {
                    uint16_t k = (uint16_t)(n - i);
                    if (k > p->cnt) k = (uint16_t)p->cnt;
                    if (p->cmd & 0x20) {
                        // 读写命令: 结果要进IN环, 受环剩余空间限制
                        uint16_t fr = (uint16_t)(((p->tx_tail - p->tx_head - 1u) & (FT_TX_RING - 1)));
                        if (k > fr) k = fr;
                        if (k == 0) break;   // IN环满, 等USB中断消化
                    }
                    mpsse_shift_block(p, d + i, k);
                    i += k;
                    p->cnt -= k;
                    if (p->cnt == 0) p->st = MS_IDLE;
                } else {
                    mpsse_feed(p, d[i]);
                    i++;
                }
            }
            if (i < n) {
                // 块没吃完(IN环满), 记住位置下轮接着来
                p->blk_pos = i;
                break;
            }
            p->rx_deal = (uint8_t)((blk + 1) % FT_BLK_NUM);
            p->blk_pos = 0;
            if (p->rx_stalled) p->rearm_out(p);

            // 块尾有数据就冲一把, 不等0x87, 降低回传延迟
            if (p->tx_head != p->tx_tail || p->want_status) p->kick_in(p);
        }
    }

    // 没有OUT流量时也要响应: 待发状态包/回传数据
    if (p->tx_head != p->tx_tail || p->want_status) {
        p->kick_in(p);
        return;
    }

    // 仿真FTDI的latency timer: 空闲超过latency毫秒就补一个纯状态包,
    // 主机的阻塞读靠这个兜底(libftdi会把状态字节剥掉)
    if ((p->bitmode != 0 || p->is_serial) && p->in_busy == 0) {
        uint32_t now = ft_tick();
        if ((uint32_t)(now - p->last_tx_tick) > (uint32_t)p->latency * FT_TICKS_PER_MS) {
            p->last_tx_tick = now;
            p->want_status = 1;
            p->kick_in(p);
        }
    }
}

/* ---------------- EP0厂商请求 ---------------- */
int ftdi_ep0_vendor(ftdi_port_t *p, uint8_t req, uint16_t val, uint16_t idx,
                    uint16_t *plen, uint8_t *buf)
{
    switch (req) {
        case 0x00:  // SIO_RESET: 0=复位 1=清RX 2=清TX
            ftdi_port_purge(p, (uint8_t)(val & 0xFF));
            p->req |= 4;  // 真FTDI清完缓冲会回一个状态包, 主机靠它收尾
            return 0;
        case 0x01:  // MODEM_CTRL
        case 0x02:  // FLOW_CTRL
        case 0x03:  // SET_BAUDRATE
        case 0x04:  // SET_DATA
        case 0x06:  // SET_EVENT_CHAR
        case 0x07:  // SET_ERROR_CHAR
            return 0;
        case 0x05:  // POLL_MODEM_STATUS, IN方向
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
        case 0x0B:  // SET_BITMODE: wValue低字节=bitmask, 高字节=mode(libftdi约定)
            p->bitmode = (uint8_t)(val >> 8);
            if (p->bitmode == 0x02) {
                p->req |= 4;  // 进MPSSE: 引擎回空闲+补发状态包, 主循环应用
            }
            return 0;
        case 0x0C:  // READ_PINS, IN方向
            buf[0] = ftdi_read_adbus();
            *plen = 1;
            return 0;
        case 0x90: {  // READ_EEPROM, IN方向, wIndex=字地址
            uint16_t w = ftdi_eeprom_word(idx);
            buf[0] = (uint8_t)w;
            buf[1] = (uint8_t)(w >> 8);
            *plen = 2;
            return 0;
        }
        case 0x91:  // WRITE_EEPROM: 装成功(不持久化)
        case 0x92:  // ERASE_EEPROM
            return 0;
        default:
            return 0xFF;
    }
}
