#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <libftdi1/ftdi.h>
int main(void)
{
    struct ftdi_context *f = ftdi_new();
    unsigned char in[4096], out[4096];
    int r, i, j, bad = 0, total = 0;
    ftdi_set_interface(f, INTERFACE_A);
    if (ftdi_usb_open_desc(f, 0x0403, 0x6042, NULL, NULL) < 0) { printf("open fail\n"); return 1; }
    ftdi_set_bitmode(f, 0, BITMODE_RESET);
    ftdi_tcioflush(f);
    ftdi_set_latency_timer(f, 2);
    ftdi_set_bitmode(f, 0xfb, BITMODE_MPSSE);
    // 环回模式下0x39读回=写出的数据
    unsigned char lb[1] = {0x84};
    ftdi_write_data(f, lb, 1);
    srand(12345);
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    int round;
    for (round = 0; round < 64; round++) {
        // 每轮: 8个 0x39命令, 每个移300字节
        int pos = 0;
        for (i = 0; i < 8; i++) {
            in[pos++] = 0x39;           // 字节 读写 LSB 负沿写
            in[pos++] = (300 - 1) & 0xff; // len-1 lo
            in[pos++] = (300 - 1) >> 8; // len hi
            for (j = 0; j < 300; j++) in[pos++] = rand() & 0xff;
        }
        in[pos++] = 0x87;               // 立即回传
        r = ftdi_write_data(f, in, pos);
        if (r != pos) { printf("write fail r=%d\n", r); return 1; }
        // 读回 8*300 字节
        int got = 0;
        while (got < 8*300) {
            r = ftdi_read_data(f, out + got, 8*300 - got);
            if (r < 0) { printf("read fail %d (%s)\n", r, ftdi_get_error_string(f)); return 1; }
            got += r;
        }
        // 校验: 环回=写出的数据
        unsigned char *sent = in + 3;
        for (i = 0; i < 8; i++) {
            if (memcmp(out + i*300, in + 3 + i*303, 300) != 0) bad++;
        }
        total += 8*300;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double sec = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec)/1e9;
    printf("stress: %d bytes, bad rounds=%d, %.2f KB/s\n", total, bad, total/1024.0/sec);
    ftdi_usb_close(f);
    return bad ? 1 : 0;
}
