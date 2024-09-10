cmake_minimum_required(VERSION 3.15)

MESSAGE(STATUS "operation system is ${CMAKE_HOST_SYSTEM_NAME}")

# project name
project(ToraDynamics VERSION 1.1.10)

# set c/cpp standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set 64bit
if(CMAKE_CL_64)
    add_compile_definitions(CMAKE_CL_64=${CMAKE_CL_64})
endif()

# set path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)

# use eigen
include_directories("E:/eigen-3.4.0")

# close MSVC micro and set arch
if(MSVC)
    add_compile_options(/w)
    add_compile_options(/bigobj)
    add_compile_options(/arch:AVX2)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE") # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE") # avoids deprecation warnings
    add_definitions("-DNOMINMAX") # do not use MSVC's min/max macros
else()
    set(EXTRA_COMPILE_FLAGS "")
    add_compile_options("-Werror")
    add_definitions("-pthread -march=native -mavx2")
endif()

include(CPack)

# compile this path
add_subdirectory(src)
