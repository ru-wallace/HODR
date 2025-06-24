CC=gcc
include config.mk
GDBUS_CFLAGS=$(shell pkg-config --cflags gio-2.0)
INCLUDE=-I$(ANDOR_DIR)/include 

GDBUS_LDFLAGS=$(shell pkg-config --libs gio-2.0)
LDFLAGS=-L$(ANDOR_DIR)/lib -landor $(GDBUS_LDFLAGS)

CFLAGS=-Wall -Wextra -O2 $(INCLUDE) $(GDBUS_CFLAGS)
TARGET=hodr
SOURCE_DIR=src
SOURCES=$(wildcard $(SOURCE_DIR)/*.c) 
DBUS_XML=$(SOURCE_DIR)/dbus_intro.xml

all: $(TARGET)
$(TARGET): $(SOURCES)
	@$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
clean:
	@rm -f $(TARGET) *.o

dbus:
	@echo "Generating dbus code..."
	@gdbus-codegen --interface-prefix hodr.server --generate-c-code control $(DBUS_XML) \
	--output-dir $(SOURCE_DIR)

debug: 
	@echo "Compiling with debug symbols..."
	$(CC) $(CFLAGS) -g -o $(TARGET) $(SOURCES) $(LDFLAGS)