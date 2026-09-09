file(GLOB HAL_SOURCES
	"*.c"
)

add_library(hal
	OBJECT
		${HAL_SOURCES}
)

target_include_directories(hal
	PUBLIC
		.
)

target_link_libraries(hal
	PRIVATE
		platform
		freertos
)
