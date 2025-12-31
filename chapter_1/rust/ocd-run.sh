#!/usr/bin/env bash
set -euo pipefail
# Flash and run via OpenOCD (ST-LINK + STM32L4x6 target). Cargo passes the ELF as $1.
openocd -f interface/stlink.cfg -f board/st_nucleo_l4.cfg -c "program $1 verify reset exit"
