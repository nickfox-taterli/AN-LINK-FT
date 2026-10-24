#include <stdio.h>
#include <libftdi1/ftdi.h>
int main(void)
{
    struct ftdi_context *f = ftdi_new();
    int r;
    unsigned char buf[16];
    if (!f) { printf("ftdi_new fail\n"); return 1; }
    r = ftdi_set_interface(f, INTERFACE_A);
    printf("set_interface %d\n", r);
    r = ftdi_usb_open_desc(f, 0x0403, 0x6042, NULL, NULL);
    printf("open %d (%s) type=%d maxpacket=%d\n", r, ftdi_get_error_string(f), f->type, f->max_packet_size);
    if (r < 0) return 1;
    r = ftdi_usb_purge_buffers(f);
    printf("purge %d (%s)\n", r, ftdi_get_error_string(f));
    r = ftdi_set_bitmode(f, 0x00, BITMODE_RESET);
    printf("bitmode_reset %d (%s)\n", r, ftdi_get_error_string(f));
    r = ftdi_set_latency_timer(f, 5);
    printf("latency %d (%s)\n", r, ftdi_get_error_string(f));
    r = ftdi_set_bitmode(f, 0xfb, BITMODE_MPSSE);
    printf("bitmode_mpsse %d (%s)\n", r, ftdi_get_error_string(f));
    r = ftdi_read_data(f, buf, 5);
    printf("read5 %d (%s) buf=%02x %02x\n", r, ftdi_get_error_string(f), buf[0], buf[1]);
    unsigned char cmd[3] = {0x86, 0x00, 0x00};
    r = ftdi_write_data(f, cmd, 3);
    printf("write_div %d\n", r);
    unsigned char imm[1] = {0x87};
    r = ftdi_write_data(f, imm, 1);
    printf("write_imm %d\n", r);
    r = ftdi_read_data(f, buf, 4);
    printf("read4 %d buf=%02x\n", r, buf[0]);
    // 环回测试: 0x3B 位模式 4位 数据0x55
    unsigned char loop[8] = {0x84, 0x3b, 0x03, 0x55, 0x85, 0x87};
    r = ftdi_write_data(f, loop, 6);
    printf("write_loop %d\n", r);
    r = ftdi_read_data(f, buf, 1);
    printf("read_loop %d buf=%02x (期望50，位读取左对齐)\n", r, buf[0]);
    ftdi_usb_close(f);
    ftdi_free(f);
    return 0;
}
