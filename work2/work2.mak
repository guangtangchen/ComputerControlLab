CC = iccavr
LIB = ilibw
CFLAGS =  -IC:\iccv7avr\include -e -D__ICC_VERSION=722 -DATMega128  -l -g -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -LC:\iccv7avr\lib -g -e:0x20000 -ucrtatmega.o -bfunc_lit:0x8c.0x20000 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:128 -beeprom:0.4096 -fihx_coff -S2
FILES = work2.o 

WORK2:	$(FILES)
	$(CC) -o WORK2 $(LFLAGS) @WORK2.lk   -lcatm128
work2.o: C:\iccv7avr\include\iom128v.h C:\iccv7avr\include\macros.h C:\iccv7avr\include\AVRdef.h .\work2.h
work2.o:	work2.c
	$(CC) -c $(CFLAGS) work2.c
