.PHONY: build
build:
	source venv/bin/activate; python -m platformio run

.PHONY: upload
upload:
	source venv/bin/activate; python -m platformio run --target upload
