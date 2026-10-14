cmake_minimum_required(VERSION 3.16)
project(esp32sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()
if(MINGW)
  add_compile_definitions(__USE_MINGW_ANSI_STDIO=1)
  add_link_options(-static)
endif()

find_package(Threads REQUIRED)

add_library(esp32sim_core STATIC
  src/core/rv32.cpp
  src/soc/esp32c3/esp32c3.cpp
  src/soc/esp32c3/peripherals.cpp
  src/soc/esp32c3/crypto.cpp
  src/soc/esp32c3/dma.cpp
  src/soc/esp32c3/gpio.cpp
  src/soc/esp32c3/gpio_signals.cpp
  src/soc/esp32c3/intmatrix.cpp
  src/soc/esp32c3/timers.cpp
  src/soc/esp32c6/esp32c6.cpp
  src/soc/esp32c6/peripherals.cpp
  src/soc/esp32c6/gpio_pads.cpp
  src/soc/esp32c6/gpio_signals.cpp
  src/soc/esp_rom.cpp
  src/soc/i2c.cpp
  src/soc/riscv_soc.cpp
  src/soc/spi_flash.cpp
  src/soc/chip.cpp
  src/host/serial_host.cpp
  src/host/tcp_serial.cpp
  src/host/pty_serial.cpp
  src/host/com_serial.cpp
  src/host/ui_server.cpp
  src/util/inflate.cpp
  src/util/md5.cpp
  src/util/json.cpp
  src/util/log.cpp
  src/util/elf.cpp
  src/boards/boards.cpp
)
target_include_directories(esp32sim_core PUBLIC src)
target_link_libraries(esp32sim_core PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(esp32sim_core PUBLIC ws2_32)
elseif(NOT APPLE)
  target_link_libraries(esp32sim_core PUBLIC util)  # openpty()
endif()

# Web UI assets (ui/, board layouts and images) compiled into the executable.
file(GLOB UI_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ui/*)
file(GLOB LAYOUT_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/boards/layouts/*.json)
file(GLOB IMAGE_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/boards/images/*)
set(UI_ASSETS "")
foreach(f IN LISTS UI_FILES)
  get_filename_component(n ${f} NAME)
  list(APPEND UI_ASSETS "${n}=${f}")
endforeach()
foreach(f IN LISTS LAYOUT_FILES)
  get_filename_component(n ${f} NAME)
  list(APPEND UI_ASSETS "layouts/${n}=${f}")
endforeach()
foreach(f IN LISTS IMAGE_FILES)
  get_filename_component(n ${f} NAME)
  list(APPEND UI_ASSETS "images/${n}=${f}")
endforeach()
string(REPLACE ";" "|" UI_ASSETS_ARG "${UI_ASSETS}")
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ui_assets.cpp
  COMMAND ${CMAKE_COMMAND} -DOUT=${CMAKE_CURRENT_BINARY_DIR}/ui_assets.cpp "-DASSETS=${UI_ASSETS_ARG}"
          -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_assets.cmake
  DEPENDS ${UI_FILES} ${LAYOUT_FILES} ${IMAGE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_assets.cmake
  COMMENT "Embedding web UI assets"
  VERBATIM)
target_sources(esp32sim_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/ui_assets.cpp)

add_executable(esp32sim src/main.cpp)
target_link_libraries(esp32sim PRIVATE esp32sim_core)
target_compile_definitions(esp32sim PRIVATE
  ESP32SIM_VERSION="${PROJECT_VERSION}"
  ESP32SIM_DEFAULT_BOARDS="${CMAKE_CURRENT_SOURCE_DIR}/boards/boards.json")

enable_testing()
add_executable(esp32sim_tests tests/unit_tests.cpp)
target_link_libraries(esp32sim_tests PRIVATE esp32sim_core)
add_test(NAME unit COMMAND esp32sim_tests)

# End-to-end test with the real esptool, if it is installed.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import esptool"
                  RESULT_VARIABLE ESPTOOL_MISSING OUTPUT_QUIET ERROR_QUIET)
  if(NOT ESPTOOL_MISSING)
    add_test(NAME esptool_flow
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/esptool_flow.py
                     --emulator $<TARGET_FILE:esp32sim>)
    # Real ESP-IDF firmware on the real ROM; skipped when roms/ has no image.
    add_test(NAME idf_hello
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/idf_hello.py
                     --emulator $<TARGET_FILE:esp32sim>)
    add_test(NAME idf_hello_c6
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/idf_hello.py
                     --emulator $<TARGET_FILE:esp32sim> --chip esp32c6)
    set_tests_properties(idf_hello idf_hello_c6 PROPERTIES SKIP_RETURN_CODE 77)
    # Web UI API against the GPIO demo firmware.
    add_test(NAME gpio_ui
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/gpio_ui.py
                     --emulator $<TARGET_FILE:esp32sim>)
    add_test(NAME gpio_ui_c6
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/gpio_ui.py
                     --emulator $<TARGET_FILE:esp32sim> --chip esp32c6)
    set_tests_properties(gpio_ui gpio_ui_c6 PROPERTIES SKIP_RETURN_CODE 77)
  endif()

  # CPU benchmark (firmware/bench): checksums, cycle timing, and the speed.
  add_test(NAME cpu_bench
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/cpu_bench.py
                   --emulator $<TARGET_FILE:esp32sim>)

  # Arduino-ESP32 compatibility suite (firmware/arduino-compat): one test per
  # chip and sketch build found in prebuilt/, plus a check that every chip
  # the emulator reports as emulated has a build, so a new chip can't skip it.
  add_test(NAME arduino_coverage
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/arduino_compat.py
                   --emulator $<TARGET_FILE:esp32sim> --coverage)
  file(GLOB ARDUINO_IMAGES CONFIGURE_DEPENDS
       ${CMAKE_CURRENT_SOURCE_DIR}/firmware/arduino-compat/prebuilt/*/arduino_compat*.bin)
  foreach(image IN LISTS ARDUINO_IMAGES)
    get_filename_component(chip_dir ${image} DIRECTORY)
    get_filename_component(chip ${chip_dir} NAME)
    get_filename_component(image_name ${image} NAME_WE)
    if(image_name STREQUAL "arduino_compat")
      set(variant uart)
    else()
      string(REPLACE "arduino_compat_" "" variant ${image_name})
    endif()
    add_test(NAME arduino_${chip}_${variant}
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/arduino_compat.py
                     --emulator $<TARGET_FILE:esp32sim> --chip ${chip} --variant ${variant})
    set_tests_properties(arduino_${chip}_${variant} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  endforeach()
endif()
