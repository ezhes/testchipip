#define BOOTADDR_REG 0x4000
#define SCRATCHPAD_BASE 0x08000000
#define SPI0_MAP_ADDR 0x20000000
#define SPI_BOOT_KEY 0x494B534F /* b"OSKI", big endian */

// boot all cores (only hart 0) and jump to main program execution
.section .text.start, "ax", @progbits
.globl _start
_start:
  li a1, 0x2000000 // base address of clint
  csrr a0, mhartid
  bnez a0, boot_core
  addi a2, a1, 4
  li a3, 1
interrupt_loop: // trigger MSIP interrupts for other harts so that they also boot
  sw a3, 0(a2)
  addi a2, a2, 4
  lw a3, -4(a2)
  bnez a3, interrupt_loop
  j boot_core_hart0

// this boot ROM doesn't know about any boot devices, so it just spins,
// waiting for the serial interface to load the program and interrupt it
.section .text.hang, "ax", @progbits
.globl _hang
_hang: // reset vector
  la a0, _start          // on MSIP interrupt, go to _start
  csrw mtvec, a0
  csrr t0, misa          // get MISA mask
  srai t0, t0, 'S' - 'A' // get 'S' supervisor bit
  andi t0, t0, 0x1
  beqz t0, 1f            // only zero mideleg if supported
  csrw mideleg, zero     // no delegation
1:
  li a0, 8               // MIE or MSIP bit
  csrw mie, a0           // set only MSIP in MIE CSR
  csrs mstatus, a0       // set MIE in mstatus CSR

wfi_loop: // wait for MSIP interrupt to start program execution
  wfi
  j wfi_loop
// jump to reset vector
.section .text.hang80, "ax", @progbits
.globl _hang80
_hang80:
  j _hang

// other harts must wait until MSIP of hart 0 is cleared to continue
boot_core:
  lw t0, 0(a1)       // load hart 0 MSIP
  bnez t0, boot_core // block until hart 0 clears own MSIP
  sll a0, a0, 2      // offset for hart MSIP
  add a1, a0, a1
boot_core_hart0:      // begin executing code at DRAM_BASE
  sw zero, 0(a1)      // clear the interrupt
boot_retry:
  auipc t0, 0
  csrw mtvec, t0      // if we fail while trying to boot, retry.
  li a0, BOOTADDR_REG // program reset vector
#if __riscv_xlen == 32
  lw a0, 0(a0)        // get boot address from bootaddr_reg SCR
#else
  ld a0, 0(a0)        // get boot address from bootaddr_reg SCR
#endif
  csrw mepc, a0       // return from interrupt to start of user program
  csrr a0, mhartid    // hartid for next level bootloader
  la a1, _dtb         // dtb address for next level bootloader
  li a2, 0x80         // set mstatus MPIE to 0
  csrc mstatus, a2
  mret

.section .text.custom_boot_stubs, "ax", @progbits
.globl _custom_boot_stubs
_custom_boot_stubs:
  /* 
  Check for [SPI0] == SPI_BOOT_KEY. Boot from [SPI0+4] if so, otherwise UART 
  To support two seperate boot modes with one custom boot pin, we implement
  a priority boot system. 
  
  If *(uint32_t *)SPI0 = SPI_BOOT_KEY (ASCII "OSKI"), we consider the SPI XIP
  region to be "blessed"/bootable and jump to SPI0 + 4
  Otherwise (i.e. SPI XIP is not blessed), we jump to BEBE to launch the 
  embedded UART shell as a backup tethered boot method.
  */
  li t0, SPI0_MAP_ADDR 
  lw t1, 0(t0)
  li t2, SPI_BOOT_KEY
  beq t1, t2, 1f
  /*  
                          ~* SPI boot failed, launch BEBE *~
  Although we've technically already released all cores by this point, since our
  SoCs only have hart0 out of reset (see Clocking System in spec) nobody else is
  running. Therefore, we don't need to worry about races through this region.

  When hart0 wants to release the cores, it can manually rewrite BOOTADR_REG
  before releasing them from reset to send them to an S2 bootloader other than
  UART.
  */

  /* setup a micro stack */
  li sp, SCRATCHPAD_BASE + 0x1000
  j bebe_main
1:
  /* SPI boot OK! Launch all harts at SPI0+4 */
  addi t0, t0, 4
  jr t0

.align 3
_dtb:
