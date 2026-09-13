cmake_minimum_required(VERSION 3.13)

# The Sagebox board is a Pico 2 W — silkscreen and the CYW43 can — and was built
# as plain pico2 for a long time. The visible cost was that GP25 is the CYW43
# chip select on a W rather than an LED, so every "LED" this repo blinked was
# silently toggling a wireless chip select and no light ever came on. That made
# the LED useless as a boot signal at exactly the moment a boot signal was
# needed.
#
# FORCE because the failure mode here is a stale cache quietly keeping pico2 in
# an existing build tree, which is the same class of silent mismatch this line
# exists to end. Pass -DPICO_BOARD=... explicitly if you really want another.
set(PICO_BOARD pico2_w CACHE STRING "Target board" FORCE)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(PICO_SDK_FETCH_FROM_GIT 1)
include(pico_sdk_import.cmake)

project(joybus_pio CXX C ASM)

pico_sdk_init()

include_directories(../include)

add_subdirectory(GamecubeConsole)
add_subdirectory(GamecubeController)
add_subdirectory(N64Console)
add_subdirectory(N64Controller)
add_subdirectory(GcDiag)
add_subdirectory(SageboxBridge)
