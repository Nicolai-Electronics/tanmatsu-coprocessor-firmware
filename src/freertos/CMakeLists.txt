file(GLOB FREERTOS_SOURCES
	"*.c"
	"portable/GCC/RISC-V/*.c"
	"portable/GCC/RISC-V/*.S"
	"portable/MemMang/heap_4.c"
)

add_library(freertos
	OBJECT
		${FREERTOS_SOURCES}
)

target_include_directories(freertos
	PUBLIC
		.
		include
		portable/GCC/RISC-V
)

target_link_libraries(freertos
	PUBLIC
		platform
)
