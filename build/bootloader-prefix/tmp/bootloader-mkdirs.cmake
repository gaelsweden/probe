# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Mod_loc/esp/esp-idf/components/bootloader/subproject"
  "C:/WRK-ESP/Probe/build/bootloader"
  "C:/WRK-ESP/Probe/build/bootloader-prefix"
  "C:/WRK-ESP/Probe/build/bootloader-prefix/tmp"
  "C:/WRK-ESP/Probe/build/bootloader-prefix/src/bootloader-stamp"
  "C:/WRK-ESP/Probe/build/bootloader-prefix/src"
  "C:/WRK-ESP/Probe/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/WRK-ESP/Probe/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
