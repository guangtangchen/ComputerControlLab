//ICC-AVR application builder : 2002-1-1 0:39:34
// Target : M128
// Crystal: 16.000Mhz

#include <iom128v.h>
#include <macros.h>
#include "demo3.h"

unsigned int timer_counter = 0;
unsigned int x_500ms = 0;
unsigned int disValue= 0;
unsigned int voltage = 0;
int dataIn=0;

#pragma interrupt_handler uart0_rx_isr:19
void uart0_rx_isr(void)
{
 //uart has received a character in UDR
   dataIn = UDR0; //从缓冲器中获取数据 
}
void uart_Putchar(char c)	// 从串口发送数据
{
	UDR0=c;
}
void port_init(void)
{
 PORTA = 0x00;
 DDRA  = 0xFF;
 PORTB = 0x00;
 DDRB  = 0x60;
 PORTC = 0xF0; //m103 output only
 DDRC  = 0xF0;
 PORTD = 0x0C;
 DDRD  = 0x08;
 PORTE = 0xC3;
 DDRE  = 0xF2;
 PORTF = 0x00;
 DDRF  = 0x00;
 PORTG = 0x00;
 DDRG  = 0x00;
}

//TIMER0 initialize - prescale:1024
// WGM: Normal
// desired value: 10mSec
// actual value:  9.984mSec (0.2%)
void timer0_init(void)
{
 TCCR0 = 0x00; //stop
 ASSR  = 0x00; //set async mode
 TCNT0 = 0x64; //set count
 OCR0  = 0x9C;
 TCCR0 = 0x07; //start timer
}

#pragma interrupt_handler timer0_ovf_isr:17
void timer0_ovf_isr(void)
{
/* timer_counter++;
 if(timer_counter >= 50)
 {
 x_500ms++;
 timer_counter = 0;
 }
 if(x_500ms >= 16)
 x_500ms = 0;*/
 TCNT0 = 0x64; //reload counter value
}

//TIMER1 initialize - prescale:8
// WGM: 14) PWM fast, TOP=ICRn
// desired value: 976.4Hz
// actual value: 976.563Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xF8; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x07;
 OCR1AL = 0xFF;
 OCR1BH = 0x07;
 OCR1BL = 0xFF;
 OCR1CH = 0x07;
 OCR1CL = 0xFF;
 ICR1H  = 0x07;
 ICR1L  = 0xFF;
 TCCR1A = 0xA2;
 TCCR1B = 0x1A; //start Timer
}

//TIMER3 initialize - prescale:8
// WGM: 14) PWM fast, TOP=ICRn
// desired value: 976.4Hz
// actual value: 976.563Hz (0.0%)
void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0xF8; //setup
 TCNT3L = 0x01;
 OCR3AH = 0x07;
 OCR3AL = 0xFF;
 OCR3BH = 0x07;
 OCR3BL = 0xFF;
 OCR3CH = 0x07;
 OCR3CL = 0xFF;
 ICR3H  = 0x07;
 ICR3L  = 0xFF;
 TCCR3A = 0x2A;
 TCCR3B = 0x1A; //start Timer
}

//UART0 initialize
// desired baud rate: 9600
// actual: baud rate:9615 (0.2%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x67; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//UART1 initialize
// desired baud rate:9600
// actual baud rate:9615 (0.2%)
// char size: 8 bit
// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x67; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x18;
}

//ADC initialize
// Conversion time: 104uS
void adc_init(void)
{
 ADCSRA = 0x00; //disable adc
 ADMUX = 0x00; //select adc input 0
 ACSR  = 0x80;
 ADCSRA = 0x87;
}

//call this routine to initialize all peripherals
void init_devices(void)
{
 //stop errant interrupts until set up
 CLI(); //disable all interrupts
 XDIV  = 0x00; //xtal divider
 XMCRA = 0x00; //external memory
 port_init();
 timer0_init();
 timer1_init();
 timer3_init();
 uart0_init();
 uart1_init();
 adc_init();

 MCUCR = 0x00;
 EICRA = 0x00; //extended ext ints
 EICRB = 0x00; //extended ext ints
 EIMSK = 0x00;
 TIMSK = 0x01; //timer interrupt sources
 ETIMSK = 0x00; //extended timer interrupt sources
 SEI(); //re-enable interrupts
 //all peripherals are now initialized
}

//
void main(void)
{
 int pwmValue[4]={0,0,0,0};
 int disValue=0;
 int key=0;
 char c_up;
 init_devices();
 //insert your functional code here...

 
 
 while(1)
 {
        c_up=(char)dataIn;
		display_v(dataIn);
  //delay_ms(200);
 }

	

}


void display_v(unsigned int value)
{	
	char i;
//	int temp;
	for(i=1;i<3;i++)
	{
	 PORTC &=~ 0xF0;
	 PORTA = 0xFF;
	 switch(i)
	 {
	  case 1:PORTC |= (1<<W4);break;
	  case 2:PORTC |= (1<<W3);break;
	  case 3:PORTC |= (1<<W2);break;
	  case 4:PORTC |= (1<<W1);break;
	 }
	  
	 PORTA = ~g_aDisplayBuf[ value - (value/16)*16 ];
	 value /= 16;
	 delay_us(250);
	}
}
void display(unsigned int value)
{	
	char i =4;
//	int temp;

	 PORTC &=~ 0xF0; 
	 PORTA = 0xFF;
	 switch(i)
	 {
	  case 1:PORTC |= (1<<W4);break;
	  case 2:PORTC |= (1<<W3);break;
	  case 3:PORTC |= (1<<W2);break;
	  case 4:PORTC |= (1<<W1);break;
	 }
	  
	 PORTA = ~g_aDisplayBuf[ value - (value/16)*16 ];
	 value /= 16;
	 delay_us(250);
	
}

void delay_us(unsigned int n)
{
 unsigned int i=0;
   while(i<n)
   {delay_1us();
    i++;
   }
}
void delay_1us(void)
{ 
  NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
}

