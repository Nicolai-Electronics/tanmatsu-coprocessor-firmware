.PHONY: all
all: build

BUILD ?= "build"
CMAKE_FLAGS ?= "-DPREFIX=riscv64-elf-"

.PHONY: build
build:
	mkdir -p ${BUILD}
	cd ${BUILD}; cmake ${CMAKE_FLAGS} ../src
	cd ${BUILD}; make

.PHONY: clean
clean:
	rm -rf ${BUILD}

.PHONY: flash
flash: build
	minichlink -w build/application/coprocessor.bin flash -b -D

.PHONY: monitor
monitor:
	minichlink -T

.PHONY: flashmonitor
flashmonitor: build
	minichlink -w build/application/coprocessor.bin flash -b -D -T