NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0020 =     32. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       00AC  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   00AC   001A =     26. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       00AC  __idata_start
       00C6  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00C6   0D8E =   3470. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0063  __text_start
       0063  __start
       0085  _exit
       0086  _port_init
       00A1  _timer0_init
       00AB  _timer0_ovf_isr
       00CE  _timer1_init
       00EB  _timer3_init
       0113  _uart0_init
       0120  _uart1_init
       0131  _adc_init
       013A  _init_devices
       0152  _init_paras
       0165  _display
       01C0  _AdToCalCode
       01F5  _CalToDaCode
       0236  _uart_Putchar
       0238  _GetKey
       024B  _controler
       030E  _main
       03FB  push_arg4
       03FD  push_arg2
       0400  mod16s
       0404  div16s
       041E  mod16u
       0420  div16u
       0421  xdiv16u
       043A  elpm32
       044E  empy16s
       045E  pop_xgsetF0FC
       0469  push_xgsetF0FC
       046B  push_xgset30FC
       046D  push_xgset00FC
       046F  push_xgset003C
       0474  neg32
       047D  asgnblk
       0495  fpsub
       0497  fpadd
       0498  fpadd_alt
       04FF  fpadd2x
       050D  fpsub1
       0517  fpsub2
       0522  fpint
       0554  uint2fp
       0556  int2fp
       055F  ulong2fp
       0561  long2fp
       0578  fpdiv1
       0581  fpdiv2
       058B  fpdiv1x
       0597  fpdiv2x
       0612  unpacks
       061A  tstR16_R19
       0622  unpacku
       063B  normalize_and_pack
       0670  pack
       0680  saveFPRegs
       068F  restoreFPRegs
       069E  fpmule
       06F1  fpmule1
       06FB  fpmule2
       0706  fpcmp
       0713  fpcmp1
       071E  fpcmp2
       072A  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   001A =     26. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __data_start
       0100  _ADChannel
       0102  _g_aDisplayBuf
       011A  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   011A   0018 =     24. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       011A  __bss_start
       011A  _mark
       011C  _numT
       011E  _count1
       0120  _count0
       0122  _chdis
       012A  _ch
       0132  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0044 =     68. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

C:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
work2.o	[ work2.c ]
<library>	[ asave.s, div16s.s, div16u.s, elpm32.s, emul16s.s, gpop5.s, gpush.s, lneg.s, struct.s, fpadd.s, fpadd2x.s ]
[ fpsub1.s, fpsub2.s, fpint.s, int2fp.s, fpdiv_new.s, fputil.s, fpmule.s, fpmule1.s, fpmule2.s, fpcmp.s ]
[ fpcmp1.s, fpcmp2.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

