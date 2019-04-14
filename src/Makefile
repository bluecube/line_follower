BUILD_DIR?=build

MCU=mk20dx256

TOOLCHAIN_DIR:=../toolchain/arm-eabi
TOOLCHAIN_PREFIX:=$(TOOLCHAIN_DIR)/bin/arm-eabi-
TARGET_CXX:=${TOOLCHAIN_PREFIX}c++
TARGET_LD:=${TOOLCHAIN_PREFIX}ld.gold
TARGET_OBJCOPY:=${TOOLCHAIN_PREFIX}objcopy

CFLAGS?=-Werror -Wall -Wextra
TARGET_CFLAGS:=${CFLAGS}

TARGET_BUILD_DIR:=${BUILD_DIR}/target
HOST_BUILD_DIR:=${BUILD_DIR}/host
TARGET_BIN_DIR:=${BUILD_DIR}/bin/target
HOST_BIN_DIR:=${BUILD_DIR}/bin/host

COMMON_CXXFILES:=$(shell find common/ -name *.cpp)

FIRMWARE_CXXFILES=${COMMON_CXXFILES} $(shell find firmware/ -name *.cpp)
FIRMWARE_OFILES:=$(addprefix ${TARGET_BUILD_DIR}/,${FIRMWARE_CXXFILES:.cpp=.o})
FIRMWARE_PRODUCT:=${TARGET_BIN_DIR}/lf.hex

SIM_CXXFILES:=${COMMON_CXXFILES} $(shell find sim/ -name *.cpp)
SIM_OFILES=$(addprefix ${HOST_BUILD_DIR}/,${SIM_CXXFILES:.cpp=.o})
SIM_PRODUCT:=${HOST_BIN_DIR}/sim

TEST_CXXFILES:=${COMMON_CXXFILES} $(shell find test/ -name *.cpp)
TEST_OFILES:=$(addprefix ${HOST_BUILD_DIR}/,${TEST_CXXFILES:.cpp=.o})
TEST_PRODUCT:=${HOST_BIN_DIR}/test

ALL_OFILES:=${FIRMWARE_OFILES} ${SIM_OFILES} ${TEST_OFILES}

.DEFAULT: all
.PHONY: all
all: ${FIRMWARE_PRODUCT} ${FIRMWARE_PRODUCT} ${SIM_PRODUCT} test

.PHONY: upload
upload: ${FIRMWARE_PRODUCT}
	@echo "TODO (upload)"

.PHONY: run_test
test: ${TEST_PRODUCT}
	${TEST_PRODUCT}

.PHONY: clean
clean:
	rm -r ${BUILD_DIR}

${FIRMWARE_PRODUCT}: ${FIRMWARE_OFILES}
	@mkdir -p ${@D}
	@echo "TODO (link firmware)"

${SIM_PRODUCT}: ${SIM_OFILES}
	@mkdir -p ${@D}
	${CXX} ${HOST_CFLAGS} -o $@ $^

${TEST_PRODUCT}: ${TEST_OFILES}
	@mkdir -p ${@D}
	${CXX} ${HOST_CFLAGS} -o $@ $^

${TARGET_BUILD_DIR}/%.o ${TARGET_BUILD_DIR}/%.d: %.cpp
	@mkdir -p ${@D}
	${TARGET_CXX} -c ${TARGET_CFLAGS} -MD -MF ${TARGET_BUILD_DIR}/$*.d -o ${TARGET_BUILD_DIR}/$*.o $^

${HOST_BUILD_DIR}/%.o ${HOST_BUILD_DIR}/%.d: %.cpp
	@mkdir -p ${@D}
	${CXX} -c ${HOST_CFLAGS} -MD -MF ${HOST_BUILD_DIR}/$*.d -o ${HOST_BUILD_DIR}/$*.o $^

%.hex: %.elf
	@mkdir -p ${@D}
	${TARGET_OBJCOPY} -O $(FORMAT) -R .eeprom -R .fuse -R .lock -R .signature $< $@

-include ${ALL_OFILES:.o=.d}
