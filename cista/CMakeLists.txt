cmake_minimum_required(VERSION 3.25)
project(cista)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE .)