# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(controller)

# SPIFFS holds only the user's profiles and is not flashed from the build: the
# web UI and the default profiles are embedded in the app (main/CMakeLists.txt),
# so neither USB flashing nor an over-the-air update touches saved profiles.
