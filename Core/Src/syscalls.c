/********************************** (C) COPYRIGHT *******************************
* File Name    : syscalls.c
* Description  : newlib裸机系统调用桩
*               这套工具链的newlib默认用Linux式ecall实现_fstat等调用,
*               裸机上ecall直接进异常死循环, 必须用桩函数盖掉
*               (串口输出_write/_sbrk在debug.c里已提供)
*******************************************************************************/
#include <sys/stat.h>
#include <errno.h>
#undef errno
extern int errno;

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;
}

int _close(int fd)
{
    (void)fd;
    return -1;
}

long _lseek(int fd, long offset, int dir)
{
    (void)fd; (void)offset; (void)dir;
    return 0;
}

int _read(int fd, char *buf, int len)
{
    (void)fd; (void)buf; (void)len;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid; (void)sig;
    errno = EINVAL;
    return -1;
}
