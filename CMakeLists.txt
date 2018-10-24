cmake_minimum_required(VERSION 3.10)
project(CG_NAIST)

set(CMAKE_CXX_STANDARD 14)

include_directories(Eigen)

add_executable(CG-NAIST main.cpp Scene.cpp Scene.h Image.h stb_image_write.h)