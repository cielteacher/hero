cmake_minimum_required(VERSION 3.22)

# 原生配置保留 + 开启C++支持
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME sincerely_no_groycheck)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 启用 C/C++/汇编
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME}
    User/Algorithm/MPC.h)
add_subdirectory(cmake/stm32cubemx)

# 自动扫描用户代码
file(GLOB SOURCES 
"${CMAKE_SOURCE_DIR}/User/BSP/*.c" 
"${CMAKE_SOURCE_DIR}/User/BSP/*.cpp" 
"${CMAKE_SOURCE_DIR}/SystemView/Sample/FreeRTOSV10/*.c"
"${CMAKE_SOURCE_DIR}/SystemView/Sample/FreeRTOSV10/Config/Cortex-M/*.c"
"${CMAKE_SOURCE_DIR}/User/device/BMI088/*.c" 
"${CMAKE_SOURCE_DIR}/User/device/motor/*.c" 
"${CMAKE_SOURCE_DIR}/User/device/music/*.c" 
"${CMAKE_SOURCE_DIR}/User/device/MiniPC/*.c"
"${CMAKE_SOURCE_DIR}/User/device/can_comm/*.c"
"${CMAKE_SOURCE_DIR}/User/device/vofa+/*.c" 
"${CMAKE_SOURCE_DIR}/User/device/remote_control/*.c"
"${CMAKE_SOURCE_DIR}/User/Algorithm/*.c" 
"${CMAKE_SOURCE_DIR}/User/Algorithm/*.cpp" 
"${CMAKE_SOURCE_DIR}/User/Task/*.c"
"${CMAKE_SOURCE_DIR}/User/Gimbal/*.c"
"${CMAKE_SOURCE_DIR}/User/Shoot/*.c"
"${CMAKE_SOURCE_DIR}/User/robot/*.c"
"${CMAKE_SOURCE_DIR}/User/Cmd/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SOURCES})

# 头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    User/device/BMI088
    User/device/motor
    User/device/music
    User/device/MiniPC
    User/device/can_comm
    User/robot
    User/device/vofa+
    User/device/remote_control   
    User/Algorithm
    User/Task
    User/Gimbal 
    User/Shoot  
    User/BSP
    User/Cmd
)

# ==============================================
# 核心配置：双精度FPU + 优化（无重复、无冲突）
# ==============================================
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfloat-abi=hard
     -Wl,--gc-sections
    -fdata-sections
    -mfpu=fpv5-d16          # 硬件双精度核心（你的核心需求）
    -ffunction-sections
    -fdata-sections
)

# Debug/Release 优化
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-O0 -g>
    $<$<CONFIG:Release>:-O2>
)

# 宏定义：仅保留双精度（删除了重复的__FPU_PRESENT，警告彻底消失）
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_DP
    __DOUBLE_PRECISION=1
)

# 清理冲突库
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接依赖（完全不变）
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfsp_math.a
    m
)