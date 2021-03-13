set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.17)

set(DEVICE msp430f6638)

set(GCC_DIR /Users/kai/opt/msp430-gcc)
set(GCC_MSP_INC_DIR ${GCC_DIR}/include)
set(GCC_BIN_DIR ${GCC_DIR}/bin)
set(GCC_INC_DIR ${GCC_DIR}/msp430-elf/include)

set(CMAKE_C_COMPILER ${GCC_BIN_DIR}/msp430-elf-gcc)
set(CMAKE_CXX_COMPILER ${GCC_BIN_DIR}/msp430-elf-g++)
set(CMAKE_ASM_COMPILER ${GCC_BIN_DIR}/msp430-elf-as)
set(CMAKE_AR ${GCC_BIN_DIR}/msp430-elf-ar)
set(CMAKE_OBJCOPY ${GCC_BIN_DIR}/msp430-elf-objcopy)
set(CMAKE_OBJDUMP ${GCC_BIN_DIR}/msp430-elf-objdump)
set(SIZE ${GCC_BIN_DIR}/msp430-elf-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(testMSP C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(-Os -mmcu=${DEVICE} -g -ffunction-sections -fdata-sections)
add_definitions(-D__${DEVICE}__ -DDEPRECATED)

include_directories(${GCC_MSP_INC_DIR} ${GCC_INC_DIR} Drivers Drivers/driverlib/5xx_6xx Drivers/inc Drivers/deprecated)

file(GLOB_RECURSE SOURCES "startup/*.*" "Core/*.*" "Drivers/*.*")

set(LINKER_SCRIPT ${GCC_MSP_INC_DIR}/${DEVICE}.ld)
#set(LINKER_SCRIPT ${GCC_MSP_INC_DIR}/msp430f6766a.ld)

#add_link_options(-g -Wl,--gc-sections)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mmcu=${DEVICE})
add_link_options(-T ${LINKER_SCRIPT} -L ${GCC_MSP_INC_DIR})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")