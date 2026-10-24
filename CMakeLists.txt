set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

cmake_minimum_required(VERSION 3.25)

# project settings
project(chftdi C ASM)

if(NOT DEFINED ENV{RISCV_TC_DIR})
    set(TC_DIR "/opt/mounriver/Toolchain/RISC-V Embedded GCC12/bin")
else()
    set(TC_DIR "$ENV{RISCV_TC_DIR}")
endif()

set(CMAKE_C_COMPILER ${TC_DIR}/riscv-wch-elf-gcc)
set(CMAKE_ASM_COMPILER ${TC_DIR}/riscv-wch-elf-gcc)
set(CMAKE_AR ${TC_DIR}/riscv-wch-elf-ar)
set(CMAKE_OBJCOPY ${TC_DIR}/riscv-wch-elf-objcopy)
set(CMAKE_OBJDUMP ${TC_DIR}/riscv-wch-elf-objdump)
set(SIZE ${TC_DIR}/riscv-wch-elf-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_STANDARD 11)

add_definitions(-DCH32V30x_D8C -DUSE_STDPERIPH_DRIVER)

add_compile_options(-march=rv32imac -mabi=ilp32 -msmall-data-limit=8)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(STATUS "Debug build")
    add_compile_options(-O0 -g)
else ()
    message(STATUS "Speed optimized build")
    # MPSSE位泵靠循环速度, 默认开优化
    add_compile_options(-O3 -g)
endif ()

include_directories(
        Core/Inc
        Drivers/CMSIS
        Drivers/CH32V30x_Peripheral/inc
        Drivers/Debug
)

file(GLOB SOURCES
        "Core/Src/*.c"
        "Drivers/CMSIS/*.c"
        "Drivers/CH32V30x_Peripheral/src/*.c"
        "Drivers/Debug/*.c"
        "Core/Startup/*.S"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Link.ld)

add_link_options(-march=rv32imac -mabi=ilp32 -msmall-data-limit=8)
add_link_options(-T ${LINKER_SCRIPT} -nostartfiles -static)
add_link_options(-Wl,-Bstatic,--gc-sections,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-Wl,--start-group -lc -lgcc -Wl,--end-group)

# -----------------------
# Create target
# -----------------------
add_executable(${PROJECT_NAME} ${SOURCES} ${LINKER_SCRIPT})
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${SIZE} $<TARGET_FILE:${PROJECT_NAME}>
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
        COMMENT "Building ${BIN_FILE}"
)
