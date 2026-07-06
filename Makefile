# MoonBoard LED — build & upload helpers
#
# Target board: Adafruit Feather ESP32 V2 (env: featheresp32v2)
#
# Examples:
#   make build              # build the firmware
#   make upload             # build + flash (set PORT=... if needed)
#   make monitor            # open the serial monitor
#   make clean              # remove build artifacts

PIO ?= pio

# Optional: override the serial port, e.g. `make upload PORT=/dev/cu.usbserial-1234`
ifdef PORT
PORT_FLAG := --upload-port $(PORT)
MONITOR_PORT_FLAG := --port $(PORT)
endif

.PHONY: build upload monitor clean help

help:
	@echo "Targets:"
	@echo "  build      build firmware"
	@echo "  upload     build + flash (PORT=... optional)"
	@echo "  monitor    open serial monitor"
	@echo "  clean      remove build artifacts"

build:
	$(PIO) run -e featheresp32v2

upload:
	$(PIO) run -e featheresp32v2 -t upload $(PORT_FLAG)

monitor:
	$(PIO) device monitor -e featheresp32v2 $(MONITOR_PORT_FLAG)

clean:
	$(PIO) run -t clean
