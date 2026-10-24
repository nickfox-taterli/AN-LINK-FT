/* Regression: partial-byte commands must not consume the next command.
 * Build: cc mpsse_bits.c -lftdi1 -o /tmp/mpsse_bits
 * Run with the debugger's JTAG interface idle; no target writes are performed.
 */
#include <libftdi1/ftdi.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    struct ftdi_context *f = ftdi_new();
    int result = 1;
    if (!f || ftdi_set_interface(f, INTERFACE_A) < 0 ||
        ftdi_usb_open(f, 0x0403, 0x6042) < 0)
        goto done;
    if (ftdi_set_bitmode(f, 0, BITMODE_RESET) < 0 ||
        ftdi_set_bitmode(f, 0x0b, BITMODE_MPSSE) < 0 ||
        ftdi_set_latency_timer(f, 1) < 0 || ftdi_tciflush(f) < 0 ||
        ftdi_tcoflush(f) < 0)
        goto close;

    /* Keep TMS high throughout software loopback, holding the TAPs in reset. */
    unsigned char setup[] = {0x80, 0x08, 0x0b, 0x84};
    if (ftdi_write_data(f, setup, sizeof(setup)) != sizeof(setup))
        goto close;
    for (int split = 0; split < 2; ++split) {
        for (int lsb = 0; lsb < 2; ++lsb) {
            for (int bits = 1; bits <= 8; ++bits) {
                unsigned char cmd[] = {
                    0x85, 0x1b, bits - 1, 0xa5, 0x84,
                    (unsigned char)(lsb ? 0x3b : 0x33), bits - 1, 0xa5,
                    0x39, 3, 0, 0x12, 0x34, 0x56, 0x78, 0x87
                };
                unsigned char expected[] = {0, 0x12, 0x34, 0x56, 0x78};
                unsigned char received[sizeof(expected)] = {0};
                expected[0] = lsb ? ((0xa5 & ((1u << bits) - 1)) << (8 - bits))
                                  : (0xa5 & (0xffu << (8 - bits)));
                int cut = split ? 3 : sizeof(cmd);
                if (ftdi_write_data(f, cmd, cut) != cut ||
                    (split && ftdi_write_data(f, cmd + cut, sizeof(cmd) - cut)
                                  != (int)sizeof(cmd) - cut))
                    goto close;
                int have = 0;
                for (int attempts = 0; have < (int)sizeof(received) && attempts < 100; ++attempts) {
                    int count = ftdi_read_data(f, received + have, sizeof(received) - have);
                    if (count < 0)
                        goto close;
                    have += count;
                }
                if (have != sizeof(received) || memcmp(received, expected, sizeof(expected))) {
                    fprintf(stderr, "FAIL split=%d lsb=%d bits=%d got=%02x expected=%02x bytes=%d\n",
                            split, lsb, bits, received[0], expected[0], have);
                    goto close;
                }
            }
        }
    }
    puts("PASS: 32 mixed bit/byte sequences, including fragmented commands");
    result = 0;
close: {
    unsigned char disable[] = {0x85, 0x80, 0x08, 0x0b};
    ftdi_write_data(f, disable, sizeof(disable));
    ftdi_usb_close(f);
}
done:
    if (result && f)
        fprintf(stderr, "%s\n", ftdi_get_error_string(f));
    if (f)
        ftdi_free(f);
    return result;
}
