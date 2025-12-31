# CMake helper for HAL headers
# Provides an INTERFACE target `hal::headers` with include directories set

add_library(hal_interface INTERFACE)

target_include_directories(hal_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
