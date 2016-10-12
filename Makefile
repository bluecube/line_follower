# Makefile adapted from https://github.com/posborne/zinc-example-lpc1768

GOAL=blink
TARGET=thumbv7em-none-eabi

OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

OUT_DIR=target/$(TARGET)/release

.PHONY: build clean listing $(OUT_FILE)

all: build listing
build: $(OUT_DIR)/$(GOAL).bin
listing: $(OUT_DIR)/$(GOAL).lst

$(OUT_DIR)/%:
	cargo build --release --target=$(TARGET) --verbose --bin=$(GOAL)

$(OUT_DIR)/%.bin: $(OUT_DIR)/%
	$(OBJCOPY) -O binary $< $@

$(OUT_DIR)/%.lst: $(OUT_DIR)/%
	$(OBJDUMP) -D $< > $@

clean:
	cargo clean

