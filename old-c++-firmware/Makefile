serial ?= /dev/ttyUSB1
platformio = source venv/bin/activate; python -m platformio

.NOTPARALLEL:

.PHONY: build
build:
	$(platformio) run

.PHONY: upload
clean:
	$(platformio) run --target clean

.PHONY: upload
upload:
	$(platformio) run --target upload --upload-port $(serial)

.PHONY: test
test:
	$(platformio) test -e native

.PHONY: terminal
terminal:
	picocom --baud 115200 $(serial)
