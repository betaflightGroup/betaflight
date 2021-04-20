
/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM);    /* end of RAM */

/* Base address where the quad spi. */
__quad_spi_start = ORIGIN(QUADSPI);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x800; /* required amount of stack */

/* Define output sections */
SECTIONS
{
  /* The vector table goes first into MAIN */
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >MAIN

  /* The program code and other data goes into MAIN */
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
  } >MAIN

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
  } >ITCM_RAM AT >MAIN

  .ARM.extab   : 
  { 
    *(.ARM.extab* .gnu.linkonce.armextab.*) 
  } >MAIN
  
  .ARM : 
  {
    __exidx_start = .;
    *(.ARM.exidx*) __exidx_end = .;
  } >MAIN

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >MAIN
  
  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >MAIN

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
  } >DTCM_RAM AT >MAIN

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
  } >DTCM_RAM

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
  } >RAM

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
  } >FASTRAM AT >MAIN

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
}
