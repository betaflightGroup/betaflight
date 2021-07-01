/*
*****************************************************************************
**
**  File        : stm32_flash_h750_exst.ld
**
**  Abstract    : Linker script for STM32H750xB Device with
**                512K AXI-RAM mapped onto AXI bus on D1 domain
**                128K SRAM1 mapped on D2 domain
**                128K SRAM2 mapped on D2 domain
**                 32K SRAM3 mapped on D2 domain
**                 64K SRAM4 mapped on D3 domain
**                 64K ITCM
**                128K DTCM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM, main RAM
0x24000000 to 0x2404FFFF  320K AXI SRAM, D1 domain
0x30000000 to 0x30003FFF   16K SRAM1, D2 domain
0x30004000 to 0x30007FFF   16K SRAM2, D2 domain
0x38000000 to 0x38003FFF   16K SRAM4, D3 domain, unused
0x38800000 to 0x38800FFF    4K BACKUP SRAM, Backup domain, unused

0x08000000 to 0x0801FFFF  128K isr vector, startup code, firmware, no config! // FLASH_Sector_0
*/

/* 

For H7 EXFL (External Flash) targets a binary is built that is placed on an external device.
The bootloader will enable the memory mapped mode on the CPU which allows code to run directly from
the external flash device.

The bootloader then executes code at the CODE_RAM address. The address of CODE_RAM is fixed to 0x90010000
and must not be changed.

The initial CODE_RAM is sized at 1MB.

*/

/* see .exst section below */
_exst_hash_size = 64;

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)    : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 128K + 192K /* 128K AXI SRAM + 192K ITCM & AXI = 320K */

    D2_RAM (rwx)      : ORIGIN = 0x30000000, LENGTH = 32K /* SRAM1 16K + SRAM2 16K */
    D3_RAM (rwx)      : ORIGIN = 0x38000000, LENGTH = 16K /* SRAM4 16K */

    BACKUP_SRAM (rwx) : ORIGIN = 0x38800000, LENGTH = 4K

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K

    OCTOSPI2 (rx)     : ORIGIN = 0x70000000, LENGTH = 256M
    OCTOSPI1 (rx)     : ORIGIN = 0x90000000, LENGTH = 256M 
    OCTOSPI1_CODE (rx): ORIGIN = ORIGIN(OCTOSPI1) + 1M, LENGTH = 1M - _exst_hash_size /* hard coded start address, as required by SPRACINGH7 boot loader, don't change! */
    EXST_HASH (rx)    : ORIGIN = ORIGIN(OCTOSPI1_CODE) + LENGTH(OCTOSPI1_CODE), LENGTH = _exst_hash_size
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", OCTOSPI1_CODE)

REGION_ALIAS("VECTAB", MAIN)

SECTIONS
{
  .DMA_RAM (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .;
    _dmaram_end__ = _edmaram;
  } >D2_RAM
  
  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .);
    _sdmarw = .;
    _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .);
    _edmarw = .;
    _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .);
    _sdmarwaxi = .;
    _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .);
    _edmarwaxi = .;
    _dmarwaxi_end__ = _edmarwaxi;
  } >RAM
}

INCLUDE "stm32_h730_common.ld"
INCLUDE "stm32_h730_common_post.ld"
INCLUDE "stm32_ram_h730_exst_post.ld"

