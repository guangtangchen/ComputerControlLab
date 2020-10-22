NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0060   0024 =     36. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0060  __lit_start
       0084  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   0084   001C =     28. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0084  __idata_start
       00A0  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00A0   0D74 =   3444. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0050  __start
       0050  __text_start
       0072  _exit
       0073  _port_init
       008E  _timer0_init
       0098  _timer0_ovf_isr
       00AC  _timer1_init
       00C9  _timer3_init
       00F1  _uart0_init
       00FE  _uart0_rx_isr
       010D  _uart1_init
       011E  _adc_init
       0127  _init_devices
       013F  _init_paras
       014C  _delay_1us
       015C  _delay_us
       016A  _delay_ms
       017A  _display
       01EE  _AdToCalCode
       0228  _CalToDaCode
       0269  _uart_Putchar
       026B  _controler
       032E  _KeyScan
       0341  _main
       040F  push_arg4
       0411  push_arg2
       0414  elpm32
       0428  empy16s
       0438  push_xgset300C
       043D  pop_xgset300C
       0442  push_xgsetF03C
       044B  pop_xgsetF03C
       0454  neg32
       045D  asgnblk
       0475  fpsub
       0477  fpadd
       0478  fpadd_alt
       04DF  fpadd2x
       04ED  fpsub1
       04F7  fpsub2
       0502  fpint
       0534  uint2fp
       0536  int2fp
       053F  ulong2fp
       0541  long2fp
       0558  fpdiv1
       0561  fpdiv2
       056B  fpdiv1x
       0577  fpdiv2x
       05F2  unpacks
       05FA  tstR16_R19
       0602  unpacku
       061B  normalize_and_pack
       0650  pack
       0660  saveFPRegs
       066F  restoreFPRegs
       067E  fpmule
       06D1  fpmule1
       06DB  fpmule2
       06E6  fpcmp
       06F3  fpcmp1
       06FE  fpcmp2
       070A  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0060   001C =     28. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0060  _ADChannel
       0060  __data_start
       0062  _dataIn
       0064  _g_aDisplayBuf
       007C  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   007C   0014 =     20. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       007C  _numT
       007C  __bss_start
       007E  _count1
       0080  _chdis
       0088  _ch
       0090  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   004C =     76. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

C:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
RS232_2.o	[ RS232_2.c ]
<library>	[ asave.s, elpm32.s, emul16s.s, gpushx.s, gpushb.s, lneg.s, struct.s, fpadd.s, fpadd2x.s, fpsub1.s, fpsub2.s ]
[ fpint.s, int2fp.s, fpdiv_new.s, fputil.s, fpmule.s, fpmule1.s, fpmule2.s, fpcmp.s, fpcmp1.s, fpcmp2.s ]


User Global Definitions

ram_end = 0xfff
hwstk_size = 0x80

User Base Address Definitions

func_lit = 0x60
data = 0x60
eeprom:0.4096

