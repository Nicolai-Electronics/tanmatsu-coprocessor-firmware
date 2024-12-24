file(GLOB PLATFORM_SOURCES
	"*.c"
)

add_library(platform
	OBJECT
		${PLATFORM_SOURCES}
)

target_include_directories(platform
	PUBLIC
		.
)

target_compile_definitions(platform
    PUBLIC
        TARGET_MCU="CH32V203"
        MCU_PACKAGE=1
        TARGET_MCU_LD=2
        TARGET_MCU_MEMORY_SPLIT=
        TARGET_MCU_PACKAGE="CH32V203C8T6"
        CH32V20x=1

)

# Generate ld file
execute_process(COMMAND ${CMAKE_C_COMPILER} -E -P -x c -DMCU_PACKAGE=1 -DTARGET_MCU_LD=2 ${CMAKE_CURRENT_LIST_DIR}/ch32v003fun.ld OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/ch32v203c8.ld OUTPUT_QUIET)
