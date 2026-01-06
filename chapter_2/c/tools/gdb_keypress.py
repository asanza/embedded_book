#!/usr/bin/env python3
"""
Simple interactive GDB-based keypress tool.

Usage: ./c/tools/gdb_keypress.py --addr 0x50000010 --mask 0x2000 [--gdb-port 1234]

Controls:
  1 - set mask
  0 - clear
  p - pulse (set, sleep pulse-time, clear)
  q - quit

This tool performs batch `arm-none-eabi-gdb` writes to the target GDB server
at the provided port. It requires QEMU to be started with `-gdb tcp::PORT`
(or shorthand `-s` for 1234).
"""

import argparse
import sys
import termios
import tty
import subprocess
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


def gdb_write(addr, val, gdb_port):
    gdb = 'arm-none-eabi-gdb'
    # Use --batch so gdb exits after writing; connect to :port
    cmd = [gdb, '--batch', '-ex', f"target remote :{gdb_port}", '-ex', f"set {{unsigned int}}{addr} = {val}", '-ex', 'detach', '-ex', 'quit']
    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
        return True, res.stdout + res.stderr
    except subprocess.CalledProcessError as e:
        return False, e.stdout + e.stderr
    except FileNotFoundError:
        return False, f"gdb not found: {gdb}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--addr', '-a', default='0x50000010', help='Memory address to write (hex)')
    parser.add_argument('--mask', '-m', default='0x2000', help='Mask value to write as hex')
    parser.add_argument('--gdb-port', '-p', type=int, default=1234, help='GDB server port')
    parser.add_argument('--pulse-time', '-t', type=float, default=0.1, help='Pulse duration (s)')
    args = parser.parse_args()

    addr = args.addr
    mask = int(args.mask, 0)
    gdb_port = args.gdb_port

    print(f"GDB keypress tool. GDB port: {gdb_port}, address: {addr}, mask: 0x{mask:X}")
    print("Controls: 1=set, 0=clear, p=pulse, q=quit")

    try:
        while True:
            ch = getch()
            if ch == 'q':
                print(); break
            if ch == '1':
                val = mask
                ok, out = gdb_write(addr, val, gdb_port)
                print(f"set -> 0x{val:X} -> ok={ok}")
            elif ch == '0':
                val = 0
                ok, out = gdb_write(addr, val, gdb_port)
                print(f"clear -> 0x{val:X} -> ok={ok}")
            elif ch == 'p':
                ok, out = gdb_write(addr, mask, gdb_port)
                print(f"pulse set -> ok={ok}")
                time.sleep(args.pulse_time)
                ok, out = gdb_write(addr, 0, gdb_port)
                print(f"pulse clear -> ok={ok}")
            else:
                continue
    except KeyboardInterrupt:
        print()

if __name__ == '__main__':
    main()
