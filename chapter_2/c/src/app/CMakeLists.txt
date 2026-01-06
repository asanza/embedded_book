# CMake for application

add_executable(app-qemu ${CMAKE_CURRENT_SOURCE_DIR}/main.c)

# Link against HAL headers interface and the qemu arch implementation
target_link_libraries(app-qemu PRIVATE arch_qemu)

target_link_options(app-qemu PRIVATE 
    -mcpu=cortex-m0plus
    -mthumb
    -gdwarf-3
    -ggdb
    -Wl,--gc-sections
    -Wl,--warn-common
    -Wl,--check-sections
    -Wl,--print-memory-usage
    --specs=rdimon.specs
    -lrdimon
)

target_include_directories(app-qemu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Custom target to run the built `app-qemu` under QEMU using the microbit machine
add_custom_target(run-qemu
  COMMAND ${QEMU_SYSTEM_ARM_EXECUTABLE} 
  -M microbit 
  -kernel $<TARGET_FILE:app-qemu> 
  -nographic
  -monitor null
  -serial null
  -semihosting-config enable=on,target=native
  -gdb tcp::1234
  DEPENDS app-qemu
  USES_TERMINAL
  COMMENT "Run app-qemu under QEMU (microbit)"
)
target_compile_definitions(app-qemu PRIVATE "QEMU")

add_executable(app-nucleo ${CMAKE_CURRENT_SOURCE_DIR}/main.c)

# Link against HAL headers interface and the qemu arch implementation
target_link_libraries(app-nucleo PRIVATE arch_nucleo)

target_link_options(app-nucleo PRIVATE 
    -mcpu=cortex-m4
    -mthumb
    -gdwarf-3
    -ggdb
    -Wl,--gc-sections
    -Wl,--warn-common
    -Wl,--check-sections
    -Wl,--print-memory-usage
    --specs=nano.specs
    --specs=nosys.specs
)

target_include_directories(app-nucleo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(app-nucleo PRIVATE "NUCLEO")

# Custom target to flash and run `app-nucleo` using OpenOCD (Nucleo-L476RG)
add_custom_target(run-nucleo
  COMMAND ${OPENOCD_EXECUTABLE}
          -f board/st_nucleo_l4.cfg
          -c "program $<TARGET_FILE:app-nucleo> verify reset exit"
  DEPENDS app-nucleo
  USES_TERMINAL
  COMMENT "Flash and run app-nucleo via OpenOCD (Nucleo-L476RG)"
)
