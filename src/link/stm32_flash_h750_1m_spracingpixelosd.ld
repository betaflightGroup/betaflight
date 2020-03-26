/*
*****************************************************************************
**
**  File        : stm32_flash_h750_1m_spracingpixelosd.ld
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
0x20000000 to 0x2001FFFF  128K DTCM
0x24000000 to 0x2407FFFF  512K AXI SRAM, D1 domain, main RAM
0x30000000 to 0x3001FFFF  128K SRAM1, D2 domain
0x30020000 to 0x3003FFFF  128K SRAM2, D2 domain
0x30040000 to 0x30047FFF   32K SRAM3, D2 domain, unused
0x38000000 to 0x3800FFFF   64K SRAM4, D3 domain, unused
0x38800000 to 0x38800FFF    4K BACKUP SRAM, Backup domain, unused

0x08000000 to 0x0801FFFF  128K isr vector, startup code, firmware, no config! // FLASH_Sector_0
*/

_spracing_pixel_osd_framebuffer_size = (360 * 288 * 2 / 8) * 2; /* (H * V * Bit-per-pixel / Bits-per-byte) * 2 framebuffers */

/* Specify the memory areas */
MEMORY
{
    FLASH                              (rx) : ORIGIN = 0x08000000, LENGTH = 1024K

    ITCM_RAM                          (rwx) : ORIGIN = 0x00000000, LENGTH = 64K - 32K
    SPRACINGPIXELOSD_CODE              (rx) : ORIGIN = ORIGIN(ITCM_RAM) + LENGTH(ITCM_RAM), LENGTH = 32K
    
    DTCM_RAM                          (rwx) : ORIGIN = 0x20000000, LENGTH = 128K - 4K
    SPRACINGPIXELOSD_DTCM_RAM         (rwx) : ORIGIN = ORIGIN(DTCM_RAM) + LENGTH(DTCM_RAM), LENGTH = 4K
        
    RAM                               (rwx) : ORIGIN = 0x24000000, LENGTH = 512K - 2K
    SPRACINGPIXELOSD_D1_RAM            (rw) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM) - 2K, LENGTH = 2K

    FRAMEBUFFER_RAM                    (rw) : ORIGIN = 0x30000000, LENGTH = _spracing_pixel_osd_framebuffer_size
    SPRACINGPIXELOSD_D2_RAM            (rw) : ORIGIN = ORIGIN(FRAMEBUFFER_RAM) + LENGTH(FRAMEBUFFER_RAM), LENGTH = 1K
    D2_RAM                            (rwx) : ORIGIN = ORIGIN(SPRACINGPIXELOSD_D2_RAM) + LENGTH(SPRACINGPIXELOSD_D2_RAM), LENGTH = 256K - LENGTH(FRAMEBUFFER_RAM)- LENGTH(SPRACINGPIXELOSD_D2_RAM) /* SRAM1 + SRAM2 */

    MEMORY_B1                          (rx) : ORIGIN = 0x60000000, LENGTH = 0K
    QUADSPI                            (rx) : ORIGIN = 0x90000000, LENGTH = 0K 
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", FLASH)

INCLUDE "stm32_h750_common.ld"

SECTIONS
{
  /* note the location of dmaram_start and dmaram_end on the following sections */
  
  .FRAMEBUFFER_RAM (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    
    PROVIDE(__frame_buffer_start = .);
    KEEP(*(.FRAMEBUFFER_RAM))
    
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADBEEF);
    
    . = ORIGIN(FRAMEBUFFER_RAM) + LENGTH(FRAMEBUFFER_RAM);
    PROVIDE(__frame_buffer_end = .);
  } >FRAMEBUFFER_RAM

  .spracingpixelosdlibrary_d2_ram (NOLOAD):
  {
    . = ALIGN(32);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADBEEF);
    . = ORIGIN(SPRACINGPIXELOSD_D2_RAM) + LENGTH(SPRACINGPIXELOSD_D2_RAM);
  } >SPRACINGPIXELOSD_D2_RAM

  .spracingpixelosdlibrary_dtcm_ram (NOLOAD):
  {
    . = ALIGN(32);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADC0DE);
    . = ORIGIN(SPRACINGPIXELOSD_DTCM_RAM) + LENGTH(SPRACINGPIXELOSD_DTCM_RAM);
  } >SPRACINGPIXELOSD_DTCM_RAM

  .DMA_RAM (NOLOAD) :
  {
    . = ALIGN(32);
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
  
  .spracingpixelosdlibrary_d1_ram (NOLOAD):
  {
    . = ALIGN(32);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xCAFECAFE);
    . = ORIGIN(SPRACINGPIXELOSD_D1_RAM) + LENGTH(SPRACINGPIXELOSD_D1_RAM);
  } >SPRACINGPIXELOSD_D1_RAM

  .spracingpixelosdlibrary_api (NOLOAD):
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__library_descriptor_start = .);
    
    LONG(0x00000000); /* empty library descriptor */
    PROVIDE_HIDDEN (__library_vtable_start = .);
    
    /* fill to end of library, so the usage is 100% */
    FILL(0xDEADBEEF);
    . = ORIGIN(SPRACINGPIXELOSD_CODE) + LENGTH(SPRACINGPIXELOSD_CODE);
  } >SPRACINGPIXELOSD_CODE
}

INCLUDE "stm32_h750_common_post.ld"
