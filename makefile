SOURCE_FOLDERS = src libs/InstrumentStation

BOARD_TAG    = mega
BOARD_SUB    = atmega2560

UPLOAD_SPEED = 57600
ARDUINO_PORT = /dev/ttyUSB1
ARDUINO_LIBS = SPI Bounce2 RF24
ARDUINO_SKETCHBOOK = /home/danny/Arduino
ARDUINO = /usr/share/arduino
ARDUINO_DIR = $(ARDUINO)
ARDMK_VENDOR = archlinux-arduino

LOCAL_C_SRCS = $(foreach dir,$(SOURCE_FOLDERS),$(wildcard $(dir)/*.c))
LOCAL_CPP_SRCS = $(foreach dir,$(SOURCE_FOLDERS),$(wildcard $(dir)/*.cpp))
include /usr/share/arduino/Arduino.mk
