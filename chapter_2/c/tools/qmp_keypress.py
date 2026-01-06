#!/usr/bin/env python3
"""
Simple QMP keypress tool: map host keys to HMP memory writes via QMP

Usage:
  ./qmp_keypress.py --socket /tmp/qmp-sock --addr 0x50000010 --mask 0x2000

Controls:
  1  set mask
  0  clear mask
  p  pulse (set, sleep 0.1s, clear)
  q  quit

This script performs a QMP handshake (qmp_capabilities) then issues
human-monitor-command requests to run HMP `mww` memory-write commands.
It talks to a UNIX QMP socket by default; you can also pass a TCP address
if your QEMU exposes QMP over TCP (not implemented here).
"""

import socket
import json
import argparse
import sys
import termios
import tty
import time


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def send_qmp(sock, sockfile, obj):
    s = json.dumps(obj) + "\n"
    sock.sendall(s.encode())
    # read a single JSON response line (QMP speaks newline-delimited JSON)
    resp = sockfile.readline()
    if not resp:
        return None
    try:
        return json.loads(resp.decode())
    except Exception:
        return resp.decode()


def detect_hmp_memwrite(sock, sockfile, addr):
    """Try a few common HMP memory-write syntaxes and return the working
    command template or None."""
    candidates = [
        "mww {addr} {val}",
        "mw {addr} {val}",
        "m {addr} {val}",
        "memwrite {addr} {val}",
        "write {addr} {val}",
    ]
    test_val = 0
    for c in candidates:
        cmd = c.format(addr=addr, val=test_val)
        req = {"execute": "human-monitor-command", "arguments": {"command-line": cmd}}
        resp = send_qmp(sock, sockfile, req)
        # If QMP returned a dict, check for explicit error or an 'unknown command'
        # message inside the 'return' field. Only accept the candidate if there
        # is no 'error' and the 'return' value does not contain 'unknown command'.
        if isinstance(resp, dict):
            if resp.get('error') is None:
                r = resp.get('return')
                if isinstance(r, str) and 'unknown command' in r.lower():
                    continue
                return c
        # Some builds return plain strings; accept responses not containing 'unknown command'
        if isinstance(resp, str):
            if 'unknown command' not in resp.lower():
                return c
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', '-s', default='/tmp/qmp-sock', help='QMP UNIX socket path')
    parser.add_argument('--tcp', '-T', default=None, help='QMP TCP host:port (overrides --socket if given)')
    parser.add_argument('--addr', '-a', default='0x50000010', help='Memory address to write (hex)')
    parser.add_argument('--mask', '-m', default='0x2000', help='Bit mask to write for set')
    parser.add_argument('--pulse-time', '-t', type=float, default=0.1, help='Pulse duration (s) for p')
    args = parser.parse_args()

    if args.tcp:
        # expect format host:port
        try:
            host, port_s = args.tcp.split(':', 1)
            port = int(port_s, 0)
        except Exception as e:
            print(f"Invalid --tcp value: {args.tcp}: {e}")
            sys.exit(1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
        except Exception as e:
            print(f"Failed to connect to QMP tcp {args.tcp}: {e}")
            sys.exit(1)
    else:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(args.socket)
        except Exception as e:
            print(f"Failed to connect to QMP socket {args.socket}: {e}")
            sys.exit(1)

    sockfile = sock.makefile('rb')

    # read greeting
    greeting = sockfile.readline()
    if greeting:
        try:
            g = json.loads(greeting.decode())
            # print('QMP greeting:', g)
        except Exception:
            pass

    # enable capabilities
    send_qmp(sock, sockfile, {"execute": "qmp_capabilities"})

    addr = args.addr
    mask = int(args.mask, 0)

    # Attempt to detect a working HMP memory-write command template.
    hmp_template = detect_hmp_memwrite(sock, sockfile, addr)
    if hmp_template is None:
        print("No supported HMP memory-write command found (mww/mw/etc).\n"
              "You can either use a QEMU build with HMP memory write commands,\n"
              "or fall back to using GDB remote memory writes (note: this halts the CPU).\n")
        # We will fall back to issuing gdb writes if gdb is available when the user presses keys.
    else:
        print(f"Using HMP memory command template: '{hmp_template}'")

    print("Connected to QMP.")
    print("Controls: 1=set, 0=clear, p=pulse, q=quit")
    try:
        while True:
            ch = getch()
            if ch == 'q':
                print()
                break
            if ch == '1':
                val = mask
                if hmp_template:
                    cmd = hmp_template.format(addr=addr, val=f"0x{val:X}")
                else:
                    cmd = None
            elif ch == '0':
                val = 0
                if hmp_template:
                    cmd = hmp_template.format(addr=addr, val=f"0x{val:X}")
                else:
                    cmd = None
            elif ch == 'p':
                if hmp_template:
                    cmd = hmp_template.format(addr=addr, val=f"0x{mask:X}")
                    send_qmp(sock, sockfile, {"execute":"human-monitor-command","arguments":{"command-line":cmd}})
                    print(f"sent: {cmd}")
                    time.sleep(args.pulse_time)
                    cmd2 = hmp_template.format(addr=addr, val=f"0x0")
                    send_qmp(sock, sockfile, {"execute":"human-monitor-command","arguments":{"command-line":cmd2}})
                    print(f"sent: {cmd2}")
                else:
                    # GDB fallback for pulse: set then clear after pulse_time
                    import subprocess
                    gdb = 'arm-none-eabi-gdb'
                    gdb_cmd_set = [gdb, '--batch', '-ex', f"target remote :1234", '-ex', f"set {{unsigned int}}{addr} = {mask}", '-ex', 'detach', '-ex', 'quit']
                    gdb_cmd_clr = [gdb, '--batch', '-ex', f"target remote :1234", '-ex', f"set {{unsigned int}}{addr} = 0", '-ex', 'detach', '-ex', 'quit']
                    try:
                        subprocess.run(gdb_cmd_set, check=True, capture_output=True, text=True)
                        print(f"gdb set: {gdb_cmd_set}")
                    except Exception as e:
                        print(f"gdb pulse set failed: {e}")
                    time.sleep(args.pulse_time)
                    try:
                        subprocess.run(gdb_cmd_clr, check=True, capture_output=True, text=True)
                        print(f"gdb clear: {gdb_cmd_clr}")
                    except Exception as e:
                        print(f"gdb pulse clear failed: {e}")
                continue
            else:
                continue
            # If we have a working HMP template, use it. Otherwise fall back
            # to using GDB remote writes which halt the CPU briefly.
            if hmp_template and cmd:
                resp = send_qmp(sock, sockfile, {"execute":"human-monitor-command","arguments":{"command-line":cmd}})
                print(f"sent: {cmd} -> {resp}")
            else:
                # GDB fallback: requires gdbserver at target (QEMU). We will
                # run arm-none-eabi-gdb in batch mode to write memory.
                # This is slower and halts the target briefly.
                import subprocess
                gdb = 'arm-none-eabi-gdb'
                gdb_cmd = [gdb, '--batch', '-ex', f"target remote :1234", '-ex', f"set {{unsigned int}}{addr} = {val}", '-ex', 'detach', '-ex', 'quit']
                try:
                    out = subprocess.run(gdb_cmd, check=True, capture_output=True, text=True)
                    print(f"gdb write done: {gdb_cmd}")
                except Exception as e:
                    print(f"gdb fallback failed: {e}")
    finally:
        sock.close()


if __name__ == '__main__':
    main()
