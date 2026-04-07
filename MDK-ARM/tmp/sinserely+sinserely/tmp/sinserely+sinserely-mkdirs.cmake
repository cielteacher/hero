# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely"
  "D:/Desktop/sincerely/MDK-ARM/tmp/1"
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely"
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/tmp"
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/src/sinserely+sinserely-stamp"
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/src"
  "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/src/sinserely+sinserely-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/src/sinserely+sinserely-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Desktop/sincerely/MDK-ARM/tmp/sinserely+sinserely/src/sinserely+sinserely-stamp${cfgdir}") # cfgdir has leading slash
endif()
