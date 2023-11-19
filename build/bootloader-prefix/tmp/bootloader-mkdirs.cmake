# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Programing/ESP32/esp/esp-idf/components/bootloader/subproject"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/tmp"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/src"
  "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Programing/ESP32/Projects/ESP32_NeoPixel_RGBW/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
