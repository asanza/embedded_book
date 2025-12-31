# CMake for QEMU arch (micro:bit)
# Builds an `arch_qemu` static library from all source files in this folder
# and exports the linker scripts placed in the `crt/` subfolder.

add_library(arch_nucleo STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/crt/startup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_timer.c
)

# Publicly depend on the HAL headers interface which this implementation
# provides concrete implementations for.
target_link_libraries(arch_nucleo PUBLIC hal_interface)

target_include_directories(arch_nucleo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/crt/linker-script-stm32l476rg.ld)

target_link_options(arch_nucleo PUBLIC -T${LINKER_SCRIPT})

message(STATUS "arch_nucleo configured with ${LINKER_SCRIPT}")
