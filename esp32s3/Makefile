# ESP32-S3 IQ source firmware, built as a RAM image for the ROM loader.
#
# Requires ESP-IDF v5.5 (for its headers, ROM linker scripts, a few clock
# sources and the vendor PHY library) and its xtensa-esp32s3-elf toolchain.
# After `. $IDF_PATH/export.sh`, a plain `make` builds build/iq-source.bin.

IDF_PATH ?= $(error IDF_PATH is not set; source ESP-IDF v5.5's export.sh)
CROSS ?= xtensa-esp32s3-elf-
ESPTOOL ?= esptool.py
BUILD ?= build

CC := $(CROSS)gcc
NM := $(CROSS)nm
OBJDUMP := $(CROSS)objdump
IDF := $(IDF_PATH)/components
PROTOCOL := ../protocol

# Header directories, as ESP-IDF itself uses them for applications and for
# its bootloader (the latter compiles the SDK clock sources below).
APP_INCLUDE_DIRS := \
	soc/esp32s3/include soc/esp32s3/register soc/include esp_common/include \
	hal/esp32s3/include hal/include hal/platform_port/include \
	esp_rom/include esp_rom/esp32s3/include esp_rom/esp32s3 \
	esp_hw_support/include esp_hw_support/port/esp32s3/include \
	esp_phy/include esp_phy/esp32s3/include xtensa/include xtensa/esp32s3/include \
	esp_system/include esp_libc/include log/include esp_timer/include
SDK_INCLUDE_DIRS := \
	esp_hw_support/include esp_hw_support/include/soc esp_hw_support/include/soc/esp32s3 \
	esp_hw_support/dma/include esp_hw_support/ldo/include esp_hw_support/debug_probe/include \
	esp_hw_support/mspi_timing_tuning/include \
	esp_hw_support/mspi_timing_tuning/tuning_scheme_impl/include \
	esp_hw_support/power_supply/include esp_hw_support/port/include \
	esp_hw_support/include/esp_private esp_hw_support/port/esp32s3 \
	esp_hw_support/port/esp32s3/include esp_hw_support/mspi_timing_tuning/port/esp32s3 \
	esp_hw_support/mspi_timing_tuning/port/esp32s3/include \
	log/include esp_rom/include esp_rom/esp32s3/include esp_rom/esp32s3/include/esp32s3 \
	esp_rom/esp32s3 esp_common/include newlib/platform_include \
	xtensa/esp32s3/include xtensa/include xtensa/deprecated_include \
	soc/include soc/esp32s3 soc/esp32s3/include soc/esp32s3/register \
	efuse/include efuse/esp32s3/include spi_flash/include \
	hal/platform_port/include hal/esp32s3/include hal/include \
	bootloader_support/include bootloader_support/bootloader_flash/include \
	bootloader_support/private_include esp_security/include \
	esp_phy/include esp_phy/esp32s3/include
APP_INCLUDES := -I. -Isrc -I$(PROTOCOL) $(addprefix -I$(IDF)/,$(APP_INCLUDE_DIRS))
SDK_INCLUDES := -I. $(addprefix -I$(IDF)/,$(SDK_INCLUDE_DIRS))

COMMON_FLAGS := -g -mlongcalls -ffunction-sections -fdata-sections
# Application code. Loop-pattern replacement is disabled so the compiler never
# inserts memset/memcpy calls into core 1's private code.
APP_CFLAGS := $(COMMON_FLAGS) -O3 -std=gnu11 -ffreestanding -fno-builtin -mtext-section-literals \
	-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror $(APP_INCLUDES)
# ESP-IDF sources, compiled as ESP-IDF compiles them for its bootloader.
SDK_CFLAGS := $(COMMON_FLAGS) -Os -std=gnu17 -freorder-blocks -fstrict-volatile-bitfields \
	-fno-builtin-memcpy -fno-builtin-memset -fno-builtin-bzero \
	-fno-jump-tables -fno-tree-switch-conversion -fno-stack-protector \
	-DBOOTLOADER_BUILD=1 -DESP_PLATFORM -DNON_OS_BUILD=1 -D_GNU_SOURCE \
	-DSOC_MMU_PAGE_SIZE=CONFIG_MMU_PAGE_SIZE -DSOC_XTAL_FREQ_MHZ=CONFIG_XTAL_FREQ \
	-Wno-unused-parameter $(SDK_INCLUDES)
ASFLAGS := -g -mlongcalls -mtext-section-literals -I$(PROTOCOL) -Isrc

APP_SOURCES := main.c platform.c radio.c capture.c libc.c
SDK_SOURCES := \
	$(IDF)/esp_hw_support/port/esp32s3/rtc_clk.c \
	$(IDF)/esp_hw_support/port/esp32s3/rtc_init.c \
	$(IDF)/esp_hw_support/port/esp32s3/rtc_sleep.c \
	$(IDF)/esp_hw_support/port/esp32s3/rtc_time.c \
	$(IDF)/log/src/noos/log_timestamp.c \
	$(IDF)/esp_phy/esp32s3/phy_init_data.c

OBJECTS := \
	$(addprefix $(BUILD)/,$(APP_SOURCES:.c=.o)) \
	$(addprefix $(BUILD)/sdk/,$(notdir $(SDK_SOURCES:.c=.o))) \
	$(BUILD)/startup.o $(BUILD)/vectors.o $(BUILD)/transmit_lane0.o $(BUILD)/transmit_lane1.o

ROM_LD := $(IDF)/esp_rom/esp32s3/ld
LINKER_SCRIPTS := -T memory.ld -T $(ROM_LD)/esp32s3.rom.ld -T $(BUILD)/rom-libc.ld \
	-T $(ROM_LD)/esp32s3.rom.libgcc.ld -T $(ROM_LD)/esp32s3.rom.api.ld \
	-T $(IDF)/soc/esp32s3/ld/esp32s3.peripherals.ld
LIBRARIES := $(IDF)/esp_phy/lib/esp32s3/libphy.a -lgcc

.PHONY: all clean
all: $(BUILD)/iq-source.bin

$(BUILD)/%.o: src/%.c $(wildcard src/*.h) $(wildcard $(PROTOCOL)/*.h) sdkconfig.h | $(BUILD)
	$(CC) $(APP_CFLAGS) -c $< -o $@

$(BUILD)/sdk/%.o: | $(BUILD)/sdk
	$(CC) $(SDK_CFLAGS) -c $(filter %/$*.c,$(SDK_SOURCES)) -o $@

$(BUILD)/startup.o: src/startup.S | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/vectors.o: src/vectors.S | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/transmit_lane%.o: src/transmit.S $(PROTOCOL)/link.h | $(BUILD)
	$(CC) $(ASFLAGS) -DTX_NAME=transmit_lane$* -DTX_SECTION=.tx_lane$* -c $< -o $@

# The ROM's memcpy/memset/memmove use ROM working memory that captures
# overwrite; drop them so libc.c's versions are linked instead.
$(BUILD)/rom-libc.ld: $(ROM_LD)/esp32s3.rom.libc.ld | $(BUILD)
	sed -E '/^(memcpy|memset|memmove) = /d' $< > $@

$(BUILD)/iq-source.elf: $(OBJECTS) $(BUILD)/rom-libc.ld memory.ld
	$(CC) -nostdlib -Wl,--gc-sections -Wl,--orphan-handling=warn -Wl,--fatal-warnings \
		-Wl,-Map,$(BUILD)/iq-source.map $(LINKER_SCRIPTS) $(OBJECTS) \
		-Wl,--start-group $(LIBRARIES) -Wl,--end-group -o $@
	$(NM) -n $@ > $(BUILD)/iq-source.sym
	@# ROM linker scripts silently override same-named functions; every
	@# symbol this firmware defines must resolve inside the image.
	@for symbol in $$($(NM) -g --defined-only $(filter $(BUILD)/%.o,$(OBJECTS)) | awk 'NF == 3 {print $$3}'); do \
		address=$$(awk -v s="$$symbol" '$$3 == s {print $$1}' $(BUILD)/iq-source.sym); \
		case $$address in 400[0-5]*) echo "error: $$symbol resolves to ROM at 0x$$address" >&2; exit 1;; esac; \
	done
	$(OBJDUMP) -d $@ > $(BUILD)/iq-source.lst

$(BUILD)/iq-source.bin: $(BUILD)/iq-source.elf
	$(ESPTOOL) --chip esp32s3 elf2image $< -o $@

$(BUILD) $(BUILD)/sdk:
	mkdir -p $@

clean:
	rm -rf $(BUILD)
