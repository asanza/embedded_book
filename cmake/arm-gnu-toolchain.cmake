# Toolchain file for ARM GNU embedded toolchain (arm-none-eabi)
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gnu-toolchain.cmake <path-to-source>

# --- User-visible options (can be overridden with -D on the cmake command line)
set(ARM_GNU_TOOLCHAIN_PREFIX "arm-none-eabi" CACHE STRING "Toolchain prefix (e.g. arm-none-eabi)")
set(ARM_GNU_TOOLCHAIN_PATH "" CACHE PATH "Path to directory containing the cross-compilation tools (bin folder)")
set(ARM_GNU_TOOLCHAIN_CPU "cortex-m4" CACHE STRING "Target CPU (e.g. cortex-m0 cortex-m3 cortex-m4 cortex-m7)")
set(ARM_GNU_TOOLCHAIN_FPU "" CACHE STRING "FPU type, e.g. fpv4-sp-d16 or empty")
set(ARM_GNU_TOOLCHAIN_FLOAT_ABI "soft" CACHE STRING "Floating point ABI: soft, softfp, hard")
set(ARM_GNU_TOOLCHAIN_MCU_FLAGS "-mthumb -mcpu=${ARM_GNU_TOOLCHAIN_CPU}" CACHE STRING "Base MCU flags")

# Optional sysroot (useful when using a packaged sysroot)
set(ARM_GNU_TOOLCHAIN_SYSROOT "" CACHE PATH "Optional sysroot path for the toolchain")

# Build type defaults
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Helper: locate(program) in PATH or provided path
function(_arm_find_tool var name)
  if(ARM_GNU_TOOLCHAIN_PATH)
    find_program(_found NAMES ${name} PATHS ${ARM_GNU_TOOLCHAIN_PATH} DOC "")
  else()
    find_program(_found NAMES ${name})
  endif()
  if(NOT _found)
    message(FATAL_ERROR "Required tool '${name}' not found. Set ARM_GNU_TOOLCHAIN_PATH or ensure it is in PATH.")
  endif()
  set(${var} "${_found}" PARENT_SCOPE)
endfunction()

# Determine executables
_arm_find_tool(ARM_GNU_C_COMPILER "${ARM_GNU_TOOLCHAIN_PREFIX}-gcc")
_arm_find_tool(ARM_GNU_CXX_COMPILER "${ARM_GNU_TOOLCHAIN_PREFIX}-g++")
_arm_find_tool(ARM_GNU_AS "${ARM_GNU_TOOLCHAIN_PREFIX}-as")
_arm_find_tool(ARM_GNU_OBJCOPY "${ARM_GNU_TOOLCHAIN_PREFIX}-objcopy")
_arm_find_tool(ARM_GNU_OBJDUMP "${ARM_GNU_TOOLCHAIN_PREFIX}-objdump")
_arm_find_tool(ARM_GNU_SIZE "${ARM_GNU_TOOLCHAIN_PREFIX}-size")
_arm_find_tool(ARM_GNU_AR "${ARM_GNU_TOOLCHAIN_PREFIX}-ar")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Set compilers
set(CMAKE_C_COMPILER "${ARM_GNU_C_COMPILER}")
set(CMAKE_CXX_COMPILER "${ARM_GNU_CXX_COMPILER}")
set(CMAKE_ASM_COMPILER "${ARM_GNU_AS}")

# Basic flags
set(ARM_BASE_FLAGS "-mthumb -mcpu=${ARM_GNU_TOOLCHAIN_CPU}")
if(ARM_GNU_TOOLCHAIN_FPU)
  list(APPEND ARM_BASE_FLAGS "-mfpu=${ARM_GNU_TOOLCHAIN_FPU}")
endif()
if(ARM_GNU_TOOLCHAIN_FLOAT_ABI)
  list(APPEND ARM_BASE_FLAGS "-mfloat-abi=${ARM_GNU_TOOLCHAIN_FLOAT_ABI}")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ARM_BASE_FLAGS} -Os -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARM_BASE_FLAGS} -Os -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# Sysroot handling
if(ARM_GNU_TOOLCHAIN_SYSROOT)
  set(CMAKE_SYSROOT ${ARM_GNU_TOOLCHAIN_SYSROOT})
  set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
endif()

# Tools as variables for convenience
set(ARM_OBJCOPY ${ARM_GNU_OBJCOPY})
set(ARM_OBJDUMP ${ARM_GNU_OBJDUMP})
set(ARM_SIZE ${ARM_GNU_SIZE})
set(ARM_AR ${ARM_GNU_AR})

# Commonly useful compile options available to users via target properties
set(ARM_COMMON_COMPILE_OPTIONS "-ffunction-sections" CACHE STRING "Common compile options for ARM targets")

# Helpful messages
message(STATUS "Using ARM GNU toolchain prefix: ${ARM_GNU_TOOLCHAIN_PREFIX}")
message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
if(ARM_GNU_TOOLCHAIN_PATH)
  message(STATUS "Toolchain path: ${ARM_GNU_TOOLCHAIN_PATH}")
endif()

# Small usage note
message(STATUS "To override CPU, run: -DARM_GNU_TOOLCHAIN_CPU=cortex-m7")
