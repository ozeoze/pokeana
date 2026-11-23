# Bare-metal EBI pattern for the NuMaker-Server-NUC980, loaded into DDR by NuWriter:
#   make && sudo ~/Claude/tools/NUC980_NuWriter_CMD/install/bin/nuwriter run.ini
CROSS := arm-none-eabi-
TACC ?= 31
CFLAGS := -mcpu=arm926ej-s -marm -O1 -g -Wall -Wextra -ffreestanding -nostdlib -DEBI_TACC=$(TACC)u $(EXTRA_CFLAGS)
LDFLAGS := -T link.ld -nostdlib -Wl,--build-id=none

ebi_pattern.bin: ebi_pattern.elf
	$(CROSS)objcopy -O binary $< $@

ebi_pattern.elf: start.S main.c link.ld
	$(CROSS)gcc $(CFLAGS) $(LDFLAGS) start.S main.c -o $@

clean:
	rm -f ebi_pattern.elf ebi_pattern.bin

.PHONY: clean
