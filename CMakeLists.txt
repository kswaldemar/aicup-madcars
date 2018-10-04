cmake_minimum_required(VERSION 3.5)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# 3rdparty libraries
add_subdirectory(3rdparty/csimplesocket)
add_subdirectory(3rdparty/loguru)
add_subdirectory(3rdparty/json)

# Project
add_subdirectory(src)