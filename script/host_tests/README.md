# 主机侧测试

## TD/FD 原厂工具回归

`vendor_tools.py` 使用未修改的 TD/FD 2026.2，自带四 TAP 链配置修正与完整 PL ID 校验。
需要连接 DR1M90 板及本下载器，执行前退出其他占用 JTAG 的工具。
测试下载 PL SRAM，暂停两颗 ARM 核读取 OCM，再恢复两核。

```sh
python3 app/script/host_tests/vendor_tools.py \
  --bit /root/code/td_dr1m90/td_dr1m90_Runs/phy_1/td_dr1m90.bit \
  --out /tmp/ch_jtag_vendor_test
```

可通过 `--td-root`、`--fd-root` 指定安装目录；`--expected-ocm-sha256` 可比对同一板卡、
同一内存状态下保存的基线。固件忽略主机分频请求，兼容字节移位固定18MHz。
结果写入输出目录的 `result.json`，原始工具日志与生成的 SVF/Tcl 同时保留。
FD 有时不输出下载计时，此时 `fd_pld_s` 为 null，其余校验照常执行。

覆盖项目：

- TD：下载、完整 PL ID、DONE，512/513/4096/65536/65537字节随机 BYPASS 全位比较。
- FD：原生 `pld load`、双核暂停/读取 PC/恢复、两次64KiB OCM读取逐字节比较。
- FD：1/7/8/9/15/16/17/31/32/33/63/64/65/4088/4096/4104/32768/524288/524293位
  随机 BYPASS 全位比较，覆盖位尾、DMA块、USB包及65536字节命令边界。

2026-10-24 实测：同一4,052,153字节bit，TD下载从101.23秒降至7.13–7.36秒，
三轮下载及FD检查全部通过。FD独立原生下载1.866936秒；64KiB OCM读取从0.859613秒
降至0.490030秒，与优化前SHA256一致。
36MHz双向移位未通过PL ID校验，因此最终选择18MHz；36MHz仅写入也没有改善下载耗时。

## libftdi 协议测试

依赖：`apt install libftdi1-dev`，编译：`gcc -O2 -o xxx xxx.c -lftdi1`。
主机驱动占用接口时，需要先解绑对应 JTAG 接口的 ftdi_sio（按实际USB路径操作）。

- `mpsse_bits.c`：32组混合位/字节命令、读回左对齐、USB拆包回归。
- `ftditest.c`：逐步open/purge/bitmode/read，控制传输体检。
- `stress.c`：环回模式批量读写校验。
- `bench2.c`：单条64KB命令计时。
- `spibench.c`：旧版分频对比工具；当前固件固定高速，分频参数不再切换快慢路径。
