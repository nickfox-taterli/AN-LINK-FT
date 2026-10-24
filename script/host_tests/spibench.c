#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libftdi1/ftdi.h>
static struct ftdi_context *f;
static int wr(const unsigned char *d, int n) { return ftdi_write_data(f, (unsigned char*)d, n); }
static int rd(unsigned char *b, int n) { int g=0,r; while(g<n){r=ftdi_read_data(f,b+g,n-g); if(r<0)return r; g+=r;} return g; }
static double bench(uint16_t div)
{
    unsigned char cfg[4] = {0x8A, 0x86, div & 0xFF, div >> 8};
    wr(cfg, 4);
    // 发1MB只写0x11, 每包65536字节, 16包
    static unsigned char buf[65536+8];
    struct timespec t0, t1;
    int i, pos;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (i = 0; i < 16; i++) {
        pos = 0;
        buf[pos++] = 0x11; buf[pos++] = 0xFF; buf[pos++] = 0xFF;
        memset(buf + 3, 0xA5, 65536);
        wr(buf, 3 + 65536);
    }
    // 轮询0x81+0x87直到引擎消化完
    unsigned char poll[2] = {0x81, 0x87};
    unsigned char rb[4];
    for (;;) {
        if (wr(poll, 2) != 2) return -1;
        if (rd(rb, 1) == 1) break;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    return (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
}
int main(void)
{
    f = ftdi_new();
    ftdi_set_interface(f, INTERFACE_A);
    if (ftdi_usb_open_desc(f, 0x0403, 0x6042, NULL, NULL) < 0) { printf("open fail\n"); return 1; }
    ftdi_set_bitmode(f, 0, BITMODE_RESET);
    ftdi_tcioflush(f);
    ftdi_set_latency_timer(f, 2);
    ftdi_set_bitmode(f, 0xfb, BITMODE_MPSSE);
    ftdi_read_data(f, (unsigned char[8]){0}, 8);
    double t6 = bench(4);    // 6MHz: 60/(5*2)=6M
    double t30 = bench(0);   // 30MHz请求 -> SPI 18MHz
    printf("1MB移位: 6MHz(位操作)=%.3fs  30MHz请求(SPI 18M)=%.3fs\n", t6, t30);
    printf("等效TCK: 6MHz档=%.2fMHz  SPI档=%.2fMHz\n", 8*1048576.0/t6/1e6, 8*1048576.0/t30/1e6);
    ftdi_usb_close(f);
    return 0;
}
