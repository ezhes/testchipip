bootrom_img = bootrom.rv64.img bootrom.rv32.img
bootrom_dump = bootrom.rv64.dump bootrom.rv32.dump

GCC=riscv64-unknown-elf-gcc
CFLAGS_RV64=-mabi=lp64 -march=rv64imac -Os -g
CFLAGS_RV32=-mabi=ilp32 -march=rv32imac -Os -g
OBJCOPY=riscv64-unknown-elf-objcopy
OBJDUMP=riscv64-unknown-elf-objdump

CSRCS := $(wildcard *.c)
SSRCS := $(wildcard *.S)
SOURCES = $(CSRCS) $(SSRCS)


default: img dump

img: $(bootrom_img)

dump: $(bootrom_dump)

%.img: %.elf
	$(OBJCOPY) -O binary --change-addresses=-0x10000 $< $@

%.rv32.elf: $(SOURCES)
	$(GCC) $(CFLAGS_RV32) -Tlinker.ld $^ -nostdlib -static -o $@

%.rv64.elf: $(SOURCES)
	$(GCC) $(CFLAGS_RV64) -Tlinker.ld $^ -nostdlib -static -o $@

%.dump: %.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -f *.elf *.dump *.img
