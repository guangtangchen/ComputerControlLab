CC = iccavr
LIB = ilibw
CFLAGS =  -IC:\iccv7avr\include -e -D__ICC_VERSION=722 -DATMega128  -l -g -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -LC:\iccv7avr\lib -g -e:0x20000 -ucrtatmega.o -bfunc_lit:0x60.0x20000 -dram_end:0xfff -bdata:0x60.0xfff -dhwstk_size:128 -beeprom:0.4096 -fihx_coff -S2
FILES = RS232_2.o 

RS232_2:	$(FILES)
	$(CC) -o RS232_2 $(LFLAGS) @RS232_2.lk   -lcatm128
RS232_2.o: C:\iccv7avr\include\iom128v.h C:\iccv7avr\include\macros.h C:\iccv7avr\include\AVRdef.h .\RS232_2.h
RS232_2.o:	RS232_2.c
	$(CC) -c $(CFLAGS) RS232_2.c
