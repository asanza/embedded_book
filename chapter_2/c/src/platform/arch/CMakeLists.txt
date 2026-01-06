set(HAL_COMMON_SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/common/hal_event.c
    ${CMAKE_CURRENT_LIST_DIR}/common/hal_system.c
)
add_subdirectory(microbit)
add_subdirectory(nucleo_l476rg)