# Drive Mad for the NumWorks calculator
#   make                      -> output/device/drivemad.nwa
#   make PLATFORM=simulator   -> output/simulator/drivemad.nwb (for the Epsilon simulator)
Q ?= @
PLATFORM ?= device
NWLINK ?= npx --yes -- nwlink
BUILD_DIR = output/$(PLATFORM)

SOURCES = main.c world.c vm.c physics.c render.c inflate.c gfx.c ui.c save.c font.c platform_eadk.c
OBJECTS = $(addprefix $(BUILD_DIR)/,$(SOURCES:.c=.o)) $(BUILD_DIR)/pack_data.o

CFLAGS = -std=gnu11 -Wall -Wno-unused-function -fsingle-precision-constant

ifeq ($(PLATFORM),device)
CC = arm-none-eabi-gcc
TARGET = $(BUILD_DIR)/drivemad.nwa
CFLAGS += $(shell $(NWLINK) eadk-cflags-device) -O2
CFLAGS += -fdata-sections -ffunction-sections -flto -fno-fat-lto-objects -fwhole-program -fvisibility=internal
LDFLAGS = $(shell $(NWLINK) eadk-ldflags-device) --specs=nano.specs
LDFLAGS += -Wl,-e,main -Wl,-u,eadk_app_name -Wl,-u,eadk_app_icon -Wl,-u,eadk_api_level -Wl,--gc-sections
LDFLAGS += -flinker-output=nolto-rel
EXTRA = $(BUILD_DIR)/icon.o
else
CC = gcc
TARGET = $(BUILD_DIR)/drivemad.nwb
CFLAGS += $(shell $(NWLINK) eadk-cflags-simulator) -O2 -DNO_STORAGE
LDFLAGS = $(shell $(NWLINK) eadk-ldflags-simulator)
EXTRA =
endif

.PHONY: build clean
build: $(TARGET)

$(TARGET): $(OBJECTS) $(EXTRA)
	@echo "LD      $@"
	$(Q) $(CC) $(CFLAGS) $^ $(LDFLAGS) -lm -o $@

$(BUILD_DIR)/%.o: src/%.c src/*.h | $(BUILD_DIR)
	@echo "CC      $<"
	$(Q) $(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/pack_data.o: src/pack_data.S src/pack.bin | $(BUILD_DIR)
	@echo "AS      $<"
	$(Q) $(CC) $(CFLAGS) -Wa,-Isrc -c $< -o $@

$(BUILD_DIR)/icon.o: src/icon.png | $(BUILD_DIR)
	@echo "ICON    $<"
	$(Q) $(NWLINK) png-icon-o $< $@

$(BUILD_DIR):
	$(Q) mkdir -p $@

clean:
	$(Q) rm -rf output
