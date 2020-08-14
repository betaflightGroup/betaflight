/*
*****************************************************************************
**
**  File        : stm32_flash_f7_split.ld
**
**  Abstract    : Common linker script for STM32 devices.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM);    /* end of RAM */

/* Base address where the config is stored. */
__config_start = ORIGIN(FLASH_CONFIG);
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x800; /* required amount of stack */

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH AT >AXIM_FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH1 AT >AXIM_FLASH1

  /* Critical program code goes into ITCM RAM */
  /* Copy specific fast-executing code to ITCM RAM */ 
  tcm_code = LOADADDR(.tcm_code); 
  .tcm_code :
  {
    . = ALIGN(4);
    tcm_code_start = .; 
    *(.tcm_code)
    *(.tcm_code*)
    . = ALIGN(4);
    tcm_code_end = .; 
  } >ITCM_RAM AT >AXIM_FLASH1

  .ARM.extab   : 
  { 
    *(.ARM.extab* .gnu.linkonce.armextab.*) 
  } >FLASH
  
  .ARM : 
  {
    __exidx_start = .;
    *(.ARM.exidx*) __exidx_end = .;
  } >FLASH AT >AXIM_FLASH

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >FLASH AT >AXIM_FLASH
  
  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >FLASH_OVERFLOW

  /* Storage for the address for the configuration section so we can grab it out of the hex file */
  .custom_defaults :
  {
    . = ALIGN(4);
    KEEP (*(.custom_defaults_start_address))
    . = ALIGN(4);
    KEEP (*(.custom_defaults_end_address))
    . = ALIGN(4);
    __custom_defaults_internal_start = .;
    *(.custom_defaults);
  } >FLASH_CUSTOM_DEFAULTS

  PROVIDE_HIDDEN (__custom_defaults_start = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? ORIGIN(FLASH_CUSTOM_DEFAULTS_EXTENDED) : __custom_defaults_internal_start);
  PROVIDE_HIDDEN (__custom_defaults_end = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? ORIGIN(FLASH_CUSTOM_DEFAULTS_EXTENDED) + LENGTH(FLASH_CUSTOM_DEFAULTS_EXTENDED) : ORIGIN(FLASH_CUSTOM_DEFAULTS) + LENGTH(FLASH_CUSTOM_DEFAULTS));

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT >AXIM_FLASH1

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(SORT_BY_ALIGNMENT(.bss*))
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* Uninitialized data section */
  . = ALIGN(4);
  .sram2 (NOLOAD) :
  {
    /* This is used by the startup in order to initialize the .sram2 secion */
    _ssram2 = .;         /* define a global symbol at sram2 start */
    __sram2_start__ = _ssram2;
    *(.sram2)
    *(SORT_BY_ALIGNMENT(.sram2*))

    . = ALIGN(4);
    _esram2 = .;         /* define a global symbol at sram2 end */
    __sram2_end__ = _esram2;
  } >SRAM2

  /* used during startup to initialized fastram_data */
  _sfastram_idata = LOADADDR(.fastram_data);

  /* Initialized FAST_DATA section for unsuspecting developers */
  .fastram_data :
  {
    . = ALIGN(4);
    _sfastram_data = .;        /* create a global symbol at data start */
    *(.fastram_data)           /* .data sections */
    *(.fastram_data*)          /* .data* sections */

    . = ALIGN(4);
    _efastram_data = .;        /* define a global symbol at data end */
  } >FASTRAM AT >AXIM_FLASH1

  . = ALIGN(4);
  .fastram_bss (NOLOAD) :
  {
    _sfastram_bss = .;
    __fastram_bss_start__ = _sfastram_bss;
    *(.fastram_bss)
    *(SORT_BY_ALIGNMENT(.fastram_bss*))

    . = ALIGN(4);
    _efastram_bss = .;
    __fastram_bss_end__ = _efastram_bss;
  } >FASTRAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  _heap_stack_end = ORIGIN(STACKRAM)+LENGTH(STACKRAM) - 8; /* 8 bytes to allow for alignment */
  _heap_stack_begin = _heap_stack_end - _Min_Stack_Size  - _Min_Heap_Size;
  . = _heap_stack_begin;
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >STACKRAM = 0xa5

  /* MEMORY_bank1 section, code must be located here explicitly            */
  /* Example: extern int foo(void) __attribute__ ((section (".mb1text"))); */
  .memory_b1_text :
  {
    *(.mb1text)        /* .mb1text sections (code) */
    *(.mb1text*)       /* .mb1text* sections (code)  */
    *(.mb1rodata)      /* read-only data (constants) */
    *(.mb1rodata*)
  } >MEMORY_B1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
