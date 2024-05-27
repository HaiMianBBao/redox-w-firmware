cmake_minimum_required(VERSION 3.20)

project(pmw3610)

add_executable(pmw3610)
target_sources(pmw3610 PRIVATE pmw3610.c main.c port_win.c)
target_include_directories(pmw3610 PRIVATE .)

