MCU=mk20dx256
TARGET=thumbv7em-none-eabi

OBJCOPY=arm-none-eabi-objcopy
UPLOAD=teensy_loader_cli -mmcu=$(MCU) -v

TARGET_OUT_DIR=target/${TARGET}/release/

${TARGET_OUT_DIR}/%.hex: $(wildcard src/**/*)
	xargo build --release --target=${TARGET} --bin $*
	arm-none-eabi-strip ${@D}/$*
	$(OBJCOPY) -O ihex ${@D}/$* $@

.PHONY: upload_blink
upload_blink: ${TARGET_OUT_DIR}/blink.hex
	${UPLOAD} $<

.PHONY: clean
clean:
	xargo clean

