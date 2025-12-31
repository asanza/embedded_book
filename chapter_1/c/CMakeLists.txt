cmake_minimum_required(VERSION 3.15)

# Default toolchain file for cross-compilation. Users can override by
# passing -DCMAKE_TOOLCHAIN_FILE=/path/to/other.cmake on the cmake command line.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/arm-gnu-toolchain.cmake" CACHE STRING "Toolchain file" FORCE)
endif()

project(embedded_book_chapter_1 C)

add_compile_options(
    -mcpu=cortex-m0plus
    -Wall
    -Os
    -gdwarf-3
    -ggdb
    -c
    -mthumb
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -Wfatal-errors
)

# Find qemu-system-arm at configure time and fail early if it's not available
find_program(QEMU_SYSTEM_ARM_EXECUTABLE NAMES qemu-system-arm)
if(NOT QEMU_SYSTEM_ARM_EXECUTABLE)
  message(FATAL_ERROR "qemu-system-arm not found in PATH. Install QEMU and ensure qemu-system-arm is available in your PATH before configuring this project.")
endif()

# Find OpenOCD for flashing the Nucleo board
find_program(OPENOCD_EXECUTABLE NAMES openocd)
if(NOT OPENOCD_EXECUTABLE)
  message(FATAL_ERROR "openocd not found in PATH. Install OpenOCD to use run-nucleo.")
endif()


# Add subdirectories
add_subdirectory(src/platform)
add_subdirectory(src/app)
