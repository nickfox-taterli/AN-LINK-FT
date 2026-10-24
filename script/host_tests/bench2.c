#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libftdi1/ftdi.h>
static struct ftdi_context *f;
static int wr(const unsigned char *d, int n) { int r = ftdi_write_data(f, (unsigned char*)d, n); if (r != n) printf("  wr=%d/%d\n", r, n); return r; }
static int rd1(unsigned char *b, int ms)
{
    // 有界读: ms毫秒内等到1字节返回1, 超时返回0
    int t;
    for (t = 0; t < ms; t += 10) {
        usleep(10000);
        if (ftdi_read_data(f, b, 1) == 1) return 1;
    }
    return 0;
}
int main(void)
{
    static unsigned char buf[65536+8];
    unsigned char rb[4];
    int i, pos;
    struct timespec t0, t1;
    f = ftdi_new();
    ftdi_set_interface(f, INTERFACE_A);
    if (ftdi_usb_open_desc(f, 0x0403, 0x6042, NULL, NULL) < 0) { printf("open fail\n"); return 1; }
    ftdi_set_bitmode(f, 0, BITMODE_RESET);
    ftdi_tcioflush(f);
    ftdi_set_latency_timer(f, 2);
    ftdi_set_bitmode(f, 0xfb, BITMODE_MPSSE);
    ftdi_read_data(f, rb, 8);
    printf("init ok\n"); fflush(stdout);

    // 测试A: 单条65536字节 0x11, div5关+div0(真SPI路径)
    unsigned char cfg[4] = {0x8B, 0x86, 0x00, 0x00};
    wr(cfg, 4);
    printf("cfg 0x8B div0 sent\n"); fflush(stdout);
    pos = 0;
    buf[pos++] = 0x11; buf[pos++] = 0xFF; buf[pos++] = 0xFF;
    memset(buf + 3, 0xA5, 65536);
    clock_gettime(CLOCK_MONOTONIC, &t0);
    wr(buf, 3 + 65536);
    printf("64KB cmd queued\n"); fflush(stdout);
    // 轮询0x81
    unsigned char poll[2] = {0x81, 0x87};
    int got = 0;
    for (i = 0; i < 40; i++) {   // 最多40x100ms
        wr(poll, 2);
        if (rd1(rb, 100)) { got = 1; break; }
        printf("  poll %d: no resp\n"); fflush(stdout);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double sec = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec)/1e9;
    printf("testA(SPI 64KB单命令): %s %.3fs ADBUS=%02x\n", got?"OK":"TIMEOUT", sec, rb[0]);
    fflush(stdout);
    ftdi_usb_close(f);
    return 0;
}
