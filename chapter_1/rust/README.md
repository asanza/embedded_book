# Chapter 1 Rust HAL demo (embedded)

Minimal `no_std` Rust take on the blink/HAL example. One app drives a GPIO pin through feature-selected implementations:
- `qemu` (default): semihosting logs + cycle-based delay for `thumbv6m` (micro:bit class).
- `nucleo`: register-level GPIO for STM32L476 + cycle-based delay for `thumbv7em`.

## Layout
- `src/hal.rs`: tiny HAL traits (`Gpio`, `Timer`).
- `src/arch/qemu.rs`: semihosting GPIO + busy delay (assumes 16 MHz).
- `src/arch/nucleo.rs`: STM32L476 GPIO writes + busy delay (assumes 4 MHz).
- `src/main.rs`: platform-neutral blink using `cortex-m-rt` `#[entry]`.
- `memory-qemu.x`, `memory-nucleo.x`: linker scripts per target.
- `.cargo/config.toml`: per-target link args; defaults to `thumbv6m-none-eabi`.

## Build
From `chapter_1/rust`:

```sh
# QEMU/micro:bit-style build (thumbv6m, default feature)
cargo build --release --target thumbv6m-none-eabi

# Nucleo-L476-style build (thumbv7em)
cargo build --release --no-default-features --features nucleo --target thumbv7em-none-eabihf
```

## Run (QEMU semihosting example)
```
qemu-system-arm -machine microbit -nographic -semihosting -kernel target/thumbv6m-none-eabi/release/chapter1-hal-demo
```

## Notes
- Delays are crude cycle waits; adjust `SYSCLK_HZ`/`CPU_HZ` in the arch modules to match your clock.
