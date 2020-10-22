NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0008 =      8. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       0094  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   0094   001E =     30. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0094  __idata_start
       00B2  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00B2   03DC =    988. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0059  __start
       0059  __text_start
       0078  _exit
       0079  _uart0_rx_isr
       0088  _uart_Putchar
       008A  _port_init
       00A5  _timer0_init
       00AF  _timer0_ovf_isr
       00B8  _timer1_init
       00D5  _timer3_init
       00FD  _uart0_init
       010A  _uart1_init
       011B  _adc_init
       0123  _init_devices
       013B  _main
       0154  _display_v
       018E  _display
       01C2  _delay_us
       01CF  _delay_1us
       01DF  empy16s
       01EF  pop_gset2
       01F1  pop_gset3
       01F3  push_gset5
       01F5  push_gset4
       01F7  push_gset3
       01F9  push_gset2
       01FB  push_gset1
       01FE  pop_gset1
       01FF  pop
       0212  asgncblk
       022B  <created procedures>
       0247  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   001E =     30. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  _ADChannel
       0100  __data_start
       0102  _key
       0104  _g_aDisplayBuf
       0114  _timer_counter
       0116  _x_500ms
       0118  _disValue
       011A  _voltage
       011C  _dataIn
       011E  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   011E   0008 =      8. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       011E  __bss_start
       011E  _ch
       0126  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   004C =     76. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

C:\PROGRA~1\icc\lib\crtatmega.o	[ crtatmega.s ]
timer0.o	[ timer0.c ]
<library>	[ emul16s.s, gpop2.s, gpop3.s, gpush.s, pmstruct.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:1.4096

