#!/usr/bin/env python3
"""DR1M90 hardware regression with the unmodified TD and FD vendor tools.

Programs volatile PL SRAM, reads OCM with both ARM cores halted, then resumes
both cores. No flash operation is issued. Requires the board's four-TAP chain.
"""

import argparse
import hashlib
import json
from pathlib import Path
import random
import re
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bit", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--td-root", type=Path, default=Path(
        "/opt/anlogic/TD_Release_2026.2/TD_Release_2026.2_NL"))
    parser.add_argument("--fd-root", type=Path, default=Path(
        "/opt/anlogic/FD_Release_2026.2/FutureDynasty"))
    parser.add_argument("--speed-khz", type=int, default=1000)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--expected-ocm-sha256")
    args = parser.parse_args()
    bit = args.bit.resolve(strict=True)
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    td = args.td_root / "bin/td.sh"
    fd = args.fd_root / "toolchain/openocd/fpsoc"

    def run(command, name, timeout=600):
        start = time.monotonic()
        proc = subprocess.run([str(x) for x in command], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=timeout,
                              cwd=out)
        elapsed = time.monotonic() - start
        (out / f"{name}.log").write_text(proc.stdout)
        # TD can print PRG-2505 even after a vector-generation error.
        if proc.returncode or re.search(r"ERROR|Error:|FATAL|Assertion", proc.stdout):
            raise RuntimeError(f"{name} failed; see {out / (name + '.log')}")
        return elapsed, proc.stdout

    generate = out / "generate.tcl"
    generate.write_text(
        f"svfgen -bit {{{bit}}} -svf {{{out}/program_raw.svf}} "
        "-chip DR1M90GEG484 -scan_mode prog_sram\n"
        f"svfgen -bit {{{bit}}} -svf {{{out}/status_raw.svf}} "
        "-chip DR1M90GEG484 -scan_mode check_status_done -no_id_check\nexit\n")
    run([td, generate], "generate")
    for name in ("program", "status"):
        text = (out / f"{name}_raw.svf").read_text()
        text = re.sub(r"//[^\n]*", "", text)
        text, hir = re.subn(r"\bHIR\s+0\s*;", "HIR 13 TDI (1FFF);", text)
        text, hdr = re.subn(r"\bHDR\s+0\s*;", "HDR 3 TDI (0);", text)
        if hir != 1 or hdr != 1:
            raise RuntimeError("Unexpected vendor SVF chain headers")
        (out / f"{name}.svf").write_text(text)

    # All four TAPs in BYPASS produce a four-bit physical shift register.
    # Compare every bit, including the 65536-byte MPSSE length boundary.
    rng = random.Random(0x307)
    svf = ["TRST OFF;", "ENDIR IDLE;", "ENDDR IDLE;", "STATE RESET;",
           "STATE IDLE;", "HIR 0;", "HDR 0;", "TIR 0;", "TDR 0;",
           "SIR 21 TDI (1FFFFF);"]
    td_lengths = [512, 513, 4096, 65536, 65537]
    for size in td_lengths:
        bits = size * 8
        value = rng.getrandbits(bits)
        expected = (value << 4) & ((1 << bits) - 1)
        svf.append(f"SDR {bits} TDI ({value:0{size * 2}x}) "
                   f"TDO ({expected:0{size * 2}x}) MASK ({'f' * (size * 2)});")
    svf.extend(["STATE RESET;", "STATE IDLE;"])
    (out / "bypass.svf").write_text("\n".join(svf) + "\n")

    program = out / "program.tcl"
    program.write_text(f"connect_cable -open -host_name localhost:{args.port}\n" +
        "".join(f"program_svf -svf {{{out}/{name}.svf}} -cable 0 "
                f"-host_name localhost:{args.port}\n"
                for name in ("program", "status", "bypass")) +
        "connect_cable -close\nexit\n")
    with (out / "hwserver.log").open("w") as log:
        server = subprocess.Popen([str(args.td_root / "bin/hwserver"), "-r",
                                   "-f", "1", "-s", str(args.port)],
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            for _ in range(100):
                if server.poll() is not None:
                    raise RuntimeError("hwserver exited before opening its port")
                try:
                    with socket.create_connection(("127.0.0.1", args.port), .1):
                        break
                except OSError:
                    time.sleep(.1)
            else:
                raise RuntimeError("Timed out waiting for hwserver")
            td_wall, text = run([td, program], "td")
            if text.count("PRG-2505") != 3:
                raise RuntimeError("Expected three successful TD SVF operations")
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()
    program_time = float(re.search(r'in\s+([\d.]+)s wall', text)[1])

    commands = ["init", f"pld load 0 {{{bit}}}",
                "targets dr1m90.core.0", "halt 2000", "echo [reg pc]",
                "targets dr1m90.core.1", "halt 2000", "echo [reg pc]",
                "echo [mdw 0xF8801454 1]",
                f"dump_image {{{out}/ocm.bin}} 0x61000000 65536",
                f"dump_image {{{out}/ocm_repeat.bin}} 0x61000000 65536",
                "resume", "targets dr1m90.core.0", "resume",
                # Automatic ARM polling changes the active instruction while
                # the raw BYPASS test owns the chain. Stop it for these scans.
                "poll off",
                "irscan dr1m90.fpga 0xff"]
    fd_lengths = [1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65,
                  4088, 4096, 4104, 32768, 524288, 524293]
    for bits in fd_lengths:
        value = rng.getrandbits(bits)
        expected = format((value << 4) & ((1 << bits) - 1), "x").lstrip("0")
        commands.extend([
            f"set got [drscan dr1m90.fpga {bits} 0x{value:0{(bits + 3) // 4}x}]",
            "if {![string equal [string trimleft [string tolower $got] 0] "
            f'"{expected}"]}} {{error "BYPASS mismatch at {bits} bits"}}',
            f'echo "PASS BYPASS {bits} bits"'])
    commands.extend(['echo "VENDOR_TEST_PASS"', "shutdown"])
    fd_script = out / "fd.tcl"
    fd_script.write_text("\n".join(commands) + "\n")
    fd_wall, text = run([fd / "bin/openocd", "-s", fd / "share/openocd/scripts",
        "-c", f"set JTAG_SPEED {args.speed_khz}", "-f", "target/anlogic/dr1m90.cfg",
        "-f", fd_script], "fd", timeout=120)
    if "VENDOR_TEST_PASS" not in text or "Programming successful" not in text:
        raise RuntimeError("FD did not complete programming and all readback tests")
    data = (out / "ocm.bin").read_bytes()
    if len(data) != 65536 or data != (out / "ocm_repeat.bin").read_bytes():
        raise RuntimeError("FD OCM repeat readback mismatch")
    digest = hashlib.sha256(data).hexdigest()
    if args.expected_ocm_sha256 and digest != args.expected_ocm_sha256:
        raise RuntimeError("FD OCM differs from the supplied baseline hash")
    load = re.search(r"to fpga device 0 in (\d+)s (\d+)us", text)
    result = dict(result="PASS", bit=str(bit), bit_bytes=bit.stat().st_size,
                  bit_sha256=hashlib.sha256(bit.read_bytes()).hexdigest(),
                  td_program_s=program_time, td_wall_s=round(td_wall, 3),
                  td_bypass_bytes=td_lengths, fd_bypass_bits=fd_lengths,
                  fd_pld_s=(int(load[1]) + int(load[2]) / 1e6) if load else None,
                  fd_wall_s=round(fd_wall, 3), fd_ocm_sha256=digest)
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
