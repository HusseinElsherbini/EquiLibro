ENTRY(Reset_Handler)

MEMORY
{
  FLASH(rx):ORIGIN =0x08000000,LENGTH =240K  /* First 240KB for code */
  USER_DATA(r):ORIGIN =0x0803C000,LENGTH =16K  /* Last 16KB of sector 5 for user data */
  SRAM(rwx):ORIGIN =0x20000000,LENGTH =64K
}

_Min_Heap_Size = 0x4000;  /* 16KB minimum guarantee for heap */
_Min_Stack_Size = 0x2000; /* 8KB minimum guarantee for stack */

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

  .heap :
  {
      . = ALIGN(8);
      PROVIDE ( end = . );
      PROVIDE ( _end = . );
      . = . + _Min_Heap_Size;
      PROVIDE ( _heap_end = . );
      . = ALIGN(8);
  } >SRAM

  .stack :
  {
      . = ALIGN(8);
      . = . + _Min_Stack_Size;
      . = ALIGN(8);
      PROVIDE( _estack = . );
  } >SRAM
  .user_data :
  {
    . = ALIGN(4);
    *(.user_data)
    . = ALIGN(4);
  } > USER_DATA
}