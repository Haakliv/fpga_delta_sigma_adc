cmake_minimum_required(VERSION 3.14)

add_subdirectory(../bsp bsp)

include(../bsp/toolchain.cmake)

project(app)

enable_language(ASM)
enable_language(C)
enable_language(CXX)

add_executable(app.elf)

target_sources(app.elf
    PRIVATE
        ../delta_sigma_adc_reader.c
)

target_include_directories(app.elf
    PRIVATE
    PUBLIC
)

target_link_libraries(app.elf
    PRIVATE
        -T "${BspLinkerScript}" -nostdlib
        "${ExtraArchiveLibraries}"
        -Wl,--start-group "${BspLibraryName}" -lc -lstdc++ -lgcc -lm -Wl,--end-group
)

# Create objdump from ELF.
set(objdump app.elf.objdump)
add_custom_command(
    OUTPUT "${objdump}"
    DEPENDS app.elf
    COMMAND "${ToolchainObjdump}" "${ToolchainObjdumpFlags}" app.elf >
            "${objdump}"
    COMMENT "Creating ${objdump}."
    VERBATIM
)
add_custom_target(create-objdump ALL DEPENDS "${objdump}")

# Report space free for stack + heap. Note that the file below is never created
# so the report is always output on build.
set(stack_report_file app.elf.stack_report)
add_custom_command(
    OUTPUT "${stack_report_file}"
    DEPENDS app.elf
    COMMAND niosv-stack-report -p "${ToolchainPrefix}" app.elf
    COMMENT "Reporting memory available for stack + heap in app.elf."
    VERBATIM
)
add_custom_target(niosv-stack-report ALL DEPENDS "${stack_report_file}")

# Generate HEX file(s) from app.elf using elf2hex tool.
# Note : If ECC Full is enabled, width of 39 is set for NiosV TCM. Otherwise, 32.
add_custom_command(
    OUTPUT "onchip_memory.hex"
    DEPENDS app.elf
    COMMAND elf2hex app.elf -o onchip_memory.hex -b 0x00000000 -w 32 -e 0x0003FFFF -r 4
    COMMENT "Creating onchip_memory.hex."
    VERBATIM
)
add_custom_target(create-hex ALL DEPENDS "onchip_memory.hex")
