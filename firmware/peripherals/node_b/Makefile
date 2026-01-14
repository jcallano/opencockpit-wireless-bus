FQBN ?= esp32:esp32:esp32s3
PORT ?= /dev/ttyACM0
BAUD ?= 115200

.PHONY: compile upload monitor

compile:
	arduino-cli compile --fqbn $(FQBN) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=$(BAUD)
