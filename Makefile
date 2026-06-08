# MoonBoard LED — build & upload helpers
#
# Two boards are supported (see platformio.ini):
#   nano    -> Arduino Nano 33 IoT  (env: nano_33_iot)
#   esp32   -> Adafruit Feather ESP32 V2 (env: featheresp32v2)
#
# Examples:
#   make esp32              # build the ESP32 firmware
#   make upload-esp32       # build + flash the ESP32 (set PORT=... if needed)
#   make monitor-esp32      # open the serial monitor
#   make nano upload-nano   # build then flash the Nano
#   make all                # build both boards
#   make clean              # remove build artifacts

PIO ?= pio

# Optional: override the serial port, e.g. `make upload-esp32 PORT=/dev/cu.usbserial-1234`
ifdef PORT
PORT_FLAG := --upload-port $(PORT)
MONITOR_PORT_FLAG := --port $(PORT)
endif

.PHONY: all nano esp32 \
        upload-nano upload-esp32 \
        monitor-nano monitor-esp32 \
        clean help

help:
	@echo "Targets:"
	@echo "  nano / esp32                build firmware for that board"
	@echo "  upload-nano / upload-esp32  build + flash (PORT=... optional)"
	@echo "  monitor-nano / monitor-esp32 open serial monitor"
	@echo "  all                         build both boards"
	@echo "  clean                       remove build artifacts"

all: nano esp32

# --- Arduino Nano 33 IoT ---
nano:
	$(PIO) run -e nano_33_iot

upload-nano:
	$(PIO) run -e nano_33_iot -t upload $(PORT_FLAG)

monitor-nano:
	$(PIO) device monitor -e nano_33_iot $(MONITOR_PORT_FLAG)

# --- Adafruit Feather ESP32 V2 ---
esp32:
	$(PIO) run -e featheresp32v2

upload-esp32:
	$(PIO) run -e featheresp32v2 -t upload $(PORT_FLAG)

monitor-esp32:
	$(PIO) device monitor -e featheresp32v2 $(MONITOR_PORT_FLAG)

clean:
	$(PIO) run -t clean
