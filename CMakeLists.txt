cmake_minimum_required(VERSION 3.22)
project(parametric_analysis_tool_kit)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(
        -g
        -Wsign-compare
        -Wunreachable-code
        -DNDEBUG
        -O3
        -Wall
)

add_executable(parametric_analysis_tool_kit main.cpp
        include/parametricAnalysis.h
        src/parametric_analysis.cpp)
