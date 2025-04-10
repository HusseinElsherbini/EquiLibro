ENTRY(Reset_Handler)

MEMORY
{
  FLASH(rx):ORIGIN =0x08000000,LENGTH =240K  /* First 240KB for code */
  SRAM(rwx):ORIGIN =0x20000000,LENGTH =64K
}

_Min_Stack_Size = 0x4000; /* 16KB minimum guarantee for stack */

SECTIONS
{

    .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH
  .text :
  {
    *(.imu_dmp_fw)
    *(.text)
    *(.text.*)
    *(.init)
    *(.fini)
	  *(.rodata)
    *(.rodata.*)
	. = ALIGN(4);
	_etext = .;
  }> FLASH

    .init_array :
  {
      . = ALIGN(4);
      PROVIDE_HIDDEN (__init_array_start = .);
      KEEP (*(SORT(.init_array.*)))
      KEEP (*(.init_array*))
      PROVIDE_HIDDEN (__init_array_end = .);
      . = ALIGN(4);
  } >FLASH

  _la_data = LOADADDR(.data); /* returns the absolute load address of given section */
  .data :
  {
    _sdata = .;
	*(.data)
  *(.data.*)
	. = ALIGN(4);
	_edata = .;
  }> SRAM AT> FLASH
  
  .bss :
  {
    _sbss = .;
	__bss_start__ = _sbss;
	*(.bss)
  *(.bss.*)
	*(COMMON)
	. = ALIGN(4);
	_ebss = .;
	__bss_end__ = _ebss;
	   . = ALIGN(4); 
	end = .;
	__end__ = .;
  }> SRAM

  .stack :
  {
      . = ALIGN(8);
      . = . + _Min_Stack_Size;
      . = ALIGN(8);
      PROVIDE( _estack = . );
  } >SRAM

}