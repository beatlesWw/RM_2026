cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME aurora-farmework)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)
#================== 纯收集 .c 文件 ==================
# 用法：
#   collect_c_sources(<out-var> <根目录1> [<根目录2> ...])
# 示例：
#   collect_c_sources(MY_C_SRCS ${CMAKE_SOURCE_DIR}/modules)
#   之后 MY_C_CSRCS 里就是所有 .c 完整路径
function(collect_c_sources OUT_VAR)
    set(_roots ${ARGN})
    set(_c_files)

    foreach(_root ${_roots})
        file(GLOB_RECURSE _tmp
             CONFIGURE_DEPENDS
             ${_root}/*.c)
        list(APPEND _c_files ${_tmp})
    endforeach()

    # 去重（可选）
    list(REMOVE_DUPLICATES _c_files)
    # 把结果还给调用者
    set(${OUT_VAR} ${_c_files} PARENT_SCOPE)
endfunction()

collect_c_sources(USER_SRC
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER

)

function(collect_h_include_dirs OUT_VAR)
    set(_roots ${ARGN})
    set(_inc_dirs)

    foreach(_root ${_roots})
        file(GLOB_RECURSE _headers
             ${_root}/*.h)
        foreach(_h ${_headers})
            get_filename_component(_dir ${_h} DIRECTORY)
            list(APPEND _inc_dirs ${_dir})
        endforeach()
    endforeach()

    list(REMOVE_DUPLICATES _inc_dirs)
    set(${OUT_VAR} ${_inc_dirs} PARENT_SCOPE)
endfunction()

collect_h_include_dirs(USER_INC_DIRS
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER
)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INC_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_compile_options(aurora-farmework PRIVATE
    #---- 二进制常量 ----
    -Wno-binary-literal          # GCC 12+ 有效；老版本会自动忽略
    #---- 未使用 ----
    -Wno-unused-parameter
    -Wno-unused-variable
    #---- 空结构体 / 空翻译单元 / pedantic ----
    -Wno-pedantic                # 关闭所有 -pedantic 子警告
    #---- 函数指针转换 ----
    -Wno-pedantic                # 已包含
    #---- 空 else 体 ----
    -Wno-empty-body
    #---- 类型限制（如 uint8_t 永远 >=0 的比较）----
    -Wno-type-limits
)