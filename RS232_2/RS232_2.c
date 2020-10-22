//ICC-AVR application builder : 2014-4-25 17:39:43
// Target : M128
// Crystal: 16.000Mhz

#include <iom128v.h>
#include <macros.h>
#include "RS232_2.h"


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
    TCNT0 = 0x64; //reload counter value
	
 	count1++;
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

#pragma interrupt_handler uart0_rx_isr:19
void uart0_rx_isr(void)
{
 //uart has received a character in UDR
    dataIn = UDR0; //从缓冲器中获取数据 
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
 ADMUX = 0x40; //select adc input 0
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


void init_paras(void)	  //变量初始化
{
    numT=6;
    count1=0;
}
//------------------------------------------------------------------------------  
void delay_1us(void)
{ 
	NOP();NOP();NOP();NOP();NOP();
	NOP();NOP();NOP();NOP();NOP();
	NOP();NOP();NOP();NOP();NOP();
}

void delay_us(unsigned int n)
{
	unsigned int i=0;
	while(i<n)
	{
		delay_1us();
		i++;
	}
}

void delay_ms(unsigned int n)
{
	unsigned int i=0;
	while(i<n)
	{
		delay_us(1000);
		i++;
	}
}
//------------------------------------------------------------------------------  
void display(unsigned int numGet,unsigned int valueSend)
{	
	// 左边两位显示串口接收到的8位数据，右边的2位显示串口发送的8位数据
	char i;
	int nn;
	
	for(i=1;i<5;i++)
	{
    	PORTC &=~ 0xF0;  
		PORTA = 0xFF;
		switch(i)
		{
		     case 1:PORTC |= (1<<W4);break;	//right
		     case 2:PORTC |= (1<<W3);break;	//right middle
		     case 3:PORTC |= (1<<W2);break;	//left middle ,no use this LED 
		     case 4:PORTC |= (1<<W1);break;	//left
		}
		
		//(1)display -- 右边的2位显示串口发送的8位数据
		if((i==1) ||(i==2))		  
		{	
			PORTA = ~g_aDisplayBuf[ valueSend - (valueSend/16)*16 ];
			valueSend /= 16;
		}
		
		//(2)display  -- 左边两位显示串口接收到的8位数据
		if((i==3)||(i==4))
		{  
			PORTA = ~g_aDisplayBuf[ numGet - (numGet/16)*16 ];
			numGet /= 16;
		}
	
		 delay_ms(1);
	}
}

float AdToCalCode(unsigned int indata)
{
    // change AD code to float 
	// indata: 0--1023，对应单片机的-10V~+10V输入，对应系统的-20V~+20V
	// return: -2--+2
    float temp;
	
	temp=(float)indata;
	
	temp=(temp-511.0)/512.0;
	
	return (2*temp);
}

unsigned int CalToDaCode(float data)
{
    // change  float to DA code 
	// data: -1--0--+1，对应单片机的-10V~+10V输出
	// return: 0--1023--2047
    float temp;
	int   ii;
	
	temp=(data+1)*1023;
	
	if(temp>2047) temp=2047;
	
	ii=(int)temp;
	
	return (ii);
}

void uart_Putchar(char c)	// 从串口发送数据
{
	UDR0=c;
}

void controler(void)	  // 控制器计算
{
    float r=0.5;
	float e;
	float y;
	float u;
	float k=0.2;		  // 比例控制增益
	int iout;
	int itemp;


	// AD 转换
	StartADC();
	while(!(ADCSRA & (1<<ADIF)));
	
	//conversion complete, read value (int) using...
	ch[ADChannel] = ADCL;            //Read 8 low bits first (important)
	ch[ADChannel] += (int)ADCH << 8; //read 2 high bits and shift into top byte
			
	itemp=ch[ADChannel];
	chdis[ADChannel]=itemp>>2;   // get high 8 to display
	
	// 输入码制变换到计算码
	y=AdToCalCode(ch[ADChannel]);
	
	// 控制器计算
	e=r-y;
	
	u=k*e;
	
	// 限幅保护
	if(u<-1)  u=-1;
	if(u>=1)   u=1;
	
	// 计算码制变换到输出码
	iout=CalToDaCode(u);
	
	// DA 输出
	OCR1AH = iout>>8;	//pwmValue[0]>>8;
	OCR1AL = iout;		//pwmValue[0];
}

unsigned int KeyScan(void)	// 获取拨码开关的值
{
	int kin=0;
	
	if(!(PINB & (1<<K1)))		kin |= 0x08;
	if(!(PINB & (1<<K2)))		kin |= 0x04;
	if(!(PING & (1<<K3)))		kin |= 0x02;
	if(!(PING & (1<<K4)))		kin |= 0x01;  
	
	return(kin);
}

void main(void)
{
   	int pwmValue[4]={0,0,0,0};
	int disValue=0;
	int transMark=0;
	int key=0;
	int itemp;
	char c_up;
	
	init_devices();	  //外设接口初始化
	
	init_paras();	  //变量初始化
	
	while(1)
	{
		display(dataIn,disValue);		

	    // (1) AD
	 	ADChannel=0;
	
		// (2)get key value
		key=KeyScan();
		
		// (3)mode choice，控制器计算
		// mode except 1 ,DA out 0V
		// case 7:  1～4通道DA输出PWM占空比为50%的测试。-> 0V。
		pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 1023;
		
		// （3-1）定时器计算，获得采样周期=0.5秒
		if(count1>=50)	// 10毫秒的中断计数50次，采样一次		  
		{
		    count1=0;
			transMark=1;
		}
			
 	    //（3-2）模态选择
		if(key==1) // mode 1 ,D(z) and out control
		{
		 //	disValue = chdis[ADChannel];// high 8, a/d first channel
	
			if(transMark==1)// 采样周期到标志
			{		   
				numT++;
				controler();	// 控制器计算及其串口输出
				c_up=(char)chdis[ADChannel];// 串口上传数据转换
	
				if(numT>15) numT=0;
					
				itemp=(numT<<4)+(numT+1);	   //test
				disValue=itemp;				   //test
				c_up=(char)itemp; 			   // test	
			}
		    pwmValue[0]=1535;	 // DA输出PWM占空比为75%的测试-> +5V。
		}
		else
		{						 // other modes  
			pwmValue[0]=1023;	 // DA输出PWM占空比为50%的测试-> 0V。
		 	numT=0;
		    OCR1AH = pwmValue[0]>>8;
		    OCR1AL = pwmValue[0];
			c_up=0;	 	
			
			disValue=254;		 //test		 
		}
		
		// (3-3)其他通道的DA输出=0
		OCR1BH = pwmValue[1]>>8;
		OCR1BL = pwmValue[1];
		OCR3BH = pwmValue[2]>>8;
		OCR3BL = pwmValue[2];
		OCR3CH = pwmValue[3]>>8;
		OCR3CL = pwmValue[3];
		
		// (4)串口发送状态，无论何种工作模态都串口发送状态	
		if(transMark==1)// 采样周期到标志
		{
		 	transMark=0;
			
			while(!(UCSR0A & (1<<UDRE0)));  //判断串口发送寄存器是否不忙
		    uart_Putchar(c_up);		
		}
		// (5)display
		display(dataIn,disValue);		
	}
}

