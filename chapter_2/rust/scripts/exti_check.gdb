# exti_check.gdb
# Helper script to inspect EXTI/SYSCFG/NVIC/GPIO state and optionally trigger a SWIER for EXTI line 13 (PC13)
# Usage (example):
#   (gdb) target remote :3333
#   (gdb) source scripts/exti_check.gdb
# The script will try to halt the target, set a breakpoint on the EXTI handler,
# dump the key registers, perform a software EXTI trigger for line 13, and
# re-dump registers so you can see what changed.

# Try to interrupt/halt the target (works for gdbserver and OpenOCD)
interrupt
monitor halt

echo "\n--- Breakpoints: EXTI handlers ---\n"
# Break on the grouped handler and on the common handler if present
set breakpoint pending on
break EXTI15_10
break exti_common_handler
set breakpoint pending off

echo "\n--- Register dump (pre-SWIER) ---\n"
printf "RCC APB2ENR (SYSCFG enable) -> \n"
x /wx 0x40021064
printf "SYSCFG EXTICR4 (EXTI lines 12..15 routing) -> \n"
x /wx 0x40010014

printf "EXTI IMR/RTSR/FTSR/SWIER/PR -> \n"
x /wx 0x40010400
x /wx 0x40010408
x /wx 0x4001040c
x /wx 0x40010410
x /wx 0x40010414

printf "GPIOC MODER/PUPDR/IDR -> \n"
x /wx 0x48000800
x /wx 0x4800080c
x /wx 0x48000810

printf "NVIC ISER / ISPR / ICPR (check IRQ40 -> ISER1 bit8) -> \n"
x /wx 0xE000E100
x /wx 0xE000E104
x /wx 0xE000E200
x /wx 0xE000E204
x /wx 0xE000E280
x /wx 0xE000E284

echo "\n--- Vector symbol addresses (if available) ---\n"
info address EXTI15_10
p &EXTI15_10

# Optional: software-trigger EXTI line 13 (PC13). This will set SWIER bit
# and generate the IRQ if EXTI/NVIC are configured. Comment out the next
# block if you don't want an automatic self-trigger.

echo "\n--- SWIER self-test: set SWIER for line 13 (0x2000) ---\n"
# Perform a memory write via OpenOCD monitor command. If you're not using
# OpenOCD/GDB server that supports `monitor mww`, run this write manually in
# your debugger: `monitor mww 0x40010410 0x00002000` (or use the appropriate
# memory-write command for your server).
monitor mww 0x40010410 0x00002000

echo "PR / NVIC pending after SWIER -> \n"
x /wx 0x40010414
x /wx 0xE000E204

# Continue so CPU can handle IRQ and hit the breakpoint
echo "\n--- Continuing to let CPU handle IRQ (breakpoint should hit if wired) ---\n"
continue

# After a handler run, re-dump the EXTI PR and NVIC pending
echo "\n--- Register dump (post-IRQ) ---\n"
x /wx 0x40010414
x /wx 0xE000E204

echo "\n--- Done. If breakpoint didn't hit, paste the above register values back to the assistant. ---\n"
