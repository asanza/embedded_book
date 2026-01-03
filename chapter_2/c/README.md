# Chapter 1 C HAL demo

A tiny C example to illustrate the Hardware Abstraction Layer (HAL) idea. The application logic lives in one file and builds twice: once against a QEMU-targeted HAL and once against an STM32 Nucleo (L476RG) HAL. The same `main()` toggles a GPIO every second; only the platform layer changes.

## What is here
- `src/app/main.c`: platform-neutral app that blinks a pin with `hal_gpio_*` and `hal_timer_*`.
- `src/platform/hal/include/`: HAL contracts (`hal_gpio.h`, `hal_timer.h`).
- `src/platform/arch/qemu/`: HAL for QEMU micro:bit (nRF51 class) with its own startup code and linker script.
- `src/platform/arch/nucleo/`: HAL for STM32L476RG Nucleo with startup code, linker script, and register-level drivers.
- `cmake/arm-gnu-toolchain.cmake`: default cross toolchain (arm-none-eabi-*). You can override via `-DCMAKE_TOOLCHAIN_FILE=...`.

## Prerequisites
- CMake >= 3.15 and Ninja (or another generator).
- ARM GNU toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, etc.).
- `qemu-system-arm` in `PATH` (checked at configure time) to run the QEMU target.
- `openocd` in `PATH` (checked at configure time) to flash/run the Nucleo target.

## Configure and build
From `chapter_1/c`:

```sh
mkdir -p build
cd build
cmake -G Ninja ..              # uses cmake/arm-gnu-toolchain.cmake by default
ninja app-qemu                 # build QEMU binary
ninja app-nucleo               # build Nucleo binary
```

Optional CMake flags:
- `-DCMAKE_TOOLCHAIN_FILE=...` to point at a different toolchain file.
- `-DCMAKE_BUILD_TYPE=Release` (or `Debug`) if you want to control optimization.

## Run on QEMU
From `chapter_1/c/build` after building:

```sh
ninja run-qemu
```

This launches `qemu-system-arm -M microbit` with semihosting enabled. The QEMU HAL writes GPIO state changes to stdout.

## Flash and run on Nucleo-L476RG
From `chapter_1/c/build` after building:

```sh
ninja run-nucleo
```

`run-nucleo` invokes OpenOCD with `board/st_nucleo_l4.cfg` to program and reset the board. The app drives pin 5 (PA5 on the Nucleo-L476RG) and uses TIM2 for millisecond delays.

## How the abstraction works
- The app only calls the HAL interfaces in `hal_gpio.h` and `hal_timer.h`.
- Each architecture provides a static library (`arch_qemu` or `arch_nucleo`) with concrete implementations plus startup code and a linker script.
- CMake links `app-qemu` against `arch_qemu` and `app-nucleo` against `arch_nucleo`, reusing the same application source.

## Troubleshooting
- If CMake stops early about a missing tool, ensure `arm-none-eabi-gcc`, `qemu-system-arm`, and `openocd` are on your `PATH`.
- To use a different board or clock setup, adjust the corresponding HAL sources and linker script under `src/platform/arch/<target>/`.
