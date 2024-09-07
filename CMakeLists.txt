cmake_minimum_required(VERSION 3.15)

MESSAGE(STATUS "operation system is ${CMAKE_HOST_SYSTEM_NAME}")

# 项目名称
project(ToraDynamics VERSION 1.1.10)

# 设置C/C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加64位程序的编译开关
if(CMAKE_CL_64)
    add_compile_definitions(CMAKE_CL_64=${CMAKE_CL_64})
endif()

# 设置程序编译生成路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/install)

# 指定的程序的依赖路径
set(GF_DEPENDENCY_INCLUDE "E:/chrono/include")

# 使用eigen库
include_directories("${GF_DEPENDENCY_INCLUDE}/eigen-3.4.0")

# 使用nlohmann_json库
include_directories("${GF_DEPENDENCY_INCLUDE}/json/single_include/")

# 使用 xtensor 库加快数据运算速度
include_directories("${GF_DEPENDENCY_INCLUDE}/xtensor/include")
include_directories("${GF_DEPENDENCY_INCLUDE}/xtl/include")
include_directories("${GF_DEPENDENCY_INCLUDE}/xtensor-blas/include")

# 使用 hdf5 库存储数据
include_directories("${GF_DEPENDENCY_INCLUDE}/hdf5/c++/src")

# 添加编译选项，使json_parse 代码可以解析chrono数据类型
add_compile_definitions(PARSE_JSON_FOR_CHRONO)

# vs 编译时关闭 msvc 自定义的 max/min 宏，使用标准库中的函数
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

# 编译src和test目录
add_subdirectory(src)
