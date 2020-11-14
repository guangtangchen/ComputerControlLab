//ICC-AVR application builder : 2020-10-20 ���� 02:40:35
// Target : M128
// Crystal: 16.000Mhz

#include <iom128v.h>
#include <macros.h>

//Signal bit definitions
#define	D0	0	//PA0
#define	D1	1	//PA1
#define	D2	2	//PA2
#define	D3	3	//PA3
#define	D4	4	//PA4
#define	D5	5	//PA5
#define	D6	6	//PA6
#define	D7	7	//PA7
#define	K1	4	//PB4
#define	PW1	5	//PB5
#define	PW2	6	//PB6
#define	K2	7	//PB7
#define	W1	4	//PC4
#define	W2	5	//PC5
#define	W3	6	//PC6
#define	W4	7	//PC7
#define	IO1	0	//PD0
#define	IO2	1	//PD1
#define	_RXD1	2	//PD2
#define	_TXD1	3	//PD3
#define	_RXD0	0	//PE0
#define	_TXD0	1	//PE1
#define	PW3	4	//PE4
#define	PW4	5	//PE5
#define	IO3	6	//PE6
#define	IO4	7	//PE7
#define	ADC0	0	//PF0
#define	ADC1	1	//PF1
#define	ADC2	2	//PF2
#define	ADC3	3	//PF3
#define	K3	3	//PG3
#define	K4	4	//PG4


//ADC
#define StartADC() ADCSRA|=0x40
int ADChannel=0;
int ch[4];
int count = 0;
int dis_value = 0;   // չʾ��ֵ
int target = 0;
int transMark = 0;  // ������־
int DataGet = 0;
int DataSend = 0;
char get ;
int init_time = 0;

//��ʾ��
unsigned char g_aDisplayBuf[16]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07, \
0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71}; //0 1 2 ...9 a b c d e f

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
 count++;   // 10msһ��

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
void uart0_rx_isr(void)   // ���ڽ��������жϷ����ӳ���ֻҪ�յ����µ�ֵ���ͻ��Զ������˳���
{
 //uart has received a character in UDR
 // dataIn = UDR0; //�ӻ������л�ȡ����
 get = UDR0;
 DataGet = (int)get;
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

//------------------------------------------------------------------------------
void delay_1us(void)
{
  NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
}

void delay_us(unsigned int n)
{
 unsigned int i=0;
   while(i<n)
   {delay_1us();
    i++;
   }
}

void delay_ms(unsigned int n)
{
 unsigned int i=0;
   while(i<n)
   {delay_us(1000);
    i++;
   }
}

//------------------------------------------------------------------------------
void display(unsigned int value)
{
	char i;
	//�߰�λ��������4
//	value = value / 4;
//	int temp;
	for(i=3;i<5;i++)
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
	 delay_ms(1);
	}
}

void display_upper(unsigned int value)
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
	 delay_ms(1);
	}
}

void uart_Putchar(char c)	// ��Ƭ���򴮿ڷ�������
{
	UDR0=c;
}

//
void main(void)
{
 //insert your functional code here...
 int pwmValue[4]={0,0,0,0};
 int disValue=0;
 int key=0;
 int u0 = 0;
 int y0 = 0;
 int u = 0;
 int y = 0;
 int inData = 0;

 init_devices();
 //insert your functional code here...



 while(1)
 {

 // ��ȡÿ��AD channel��ֵ
  for(ADChannel=0;ADChannel<4;ADChannel++)
  {
   StartADC();
   while(!(ADCSRA & (1<<ADIF)));
   //conversion complete, read value (int) using...
   ch[ADChannel] = ADCL;            //Read 8 low bits first (important)
   ch[ADChannel] += (int)ADCH << 8; //read 2 high bits and shift into top byte
   ADMUX = 0x41 + ADChannel;
  }
  ADMUX = 0x40;	//reset ADC channel


  // z��������
  // code here

 // DataGet:�����·�����Ƭ����ָ��ֵ
 // target:AD���յ���ֵ
 // inDataʱDataGet��Χת��Ϊ-511��512��ֵ
 // �·�ָ�����Ϊ256����Ӧ1024����һ���ı��Ĺ�ϵ����80H��Ӧ0����
//  inData = DataGet * 4 - 511;
//  u = DataGet - target;
//  y = u - 0.6 * u0 + 0.2 * y0;
//  y0 = y;
//  u0 = u


  // ��ȡ���룬������ԵĴ���
  key=0;
  if(!(PINB & (1<<K1)))
      key |= 0x08;
  if(!(PINB & (1<<K2)))
  	  key |= 0x04;
  if(!(PING & (1<<K3)))
      key |= 0x02;
  if(!(PING & (1<<K4)))
      key |= 0x01;
  switch(key)
  { // ��Բ���Ĳ�ֵͬ��������ͬ����
   case 0:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 0;break;
   case 1:disValue = ch[0];break;
   case 2:disValue = ch[1];break;
   case 3:disValue = ch[2];break;
   case 4:disValue = ch[3];break;
   case 5:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 0;break;
   case 6:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 511;break;
   case 7:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 1023;break;
   case 8:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 1535;break;
   case 9:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 2047;break;
   case 10:pwmValue[0] = ch[0]*2.001;
   			pwmValue[1] = ch[1]*2.001;
			pwmValue[2] = ch[2]*2.001;
			pwmValue[3] = ch[3]*2.001;break;
   case 11:pwmValue[0] = 0; // 11��ʱ�򣬵�һ��DAΪ0v��Ȼ��������Ϊ����ֵ
   			pwmValue[1] = target;
			pwmValue[2] = target;
			pwmValue[3] = target;break;
   default:pwmValue[0] = pwmValue[1] = pwmValue[2] = pwmValue[3] = 0;
  }
  // ��AD���ֵ
  OCR1AH = pwmValue[0]>>8;
  OCR1AL = pwmValue[0];
  OCR1BH = pwmValue[1]>>8;
  OCR1BL = pwmValue[1];
  OCR3BH = pwmValue[2]>>8;
  OCR3BL = pwmValue[2];
  OCR3CH = pwmValue[3]>>8;
  OCR3CL = pwmValue[3];


  // 10ms�ж�һ�Σ�50�ξ���500ms�������ǵ�һ������
  if(count>=50){
       //program here
  	 dis_value++;
  	 count = 0;
  	 transMark=1;  // ������־

  	 // �涨һ����ʼ��ʱ�䣬ǰ2S�����·���ֵ��Ч����Ϊ0(����Ӧ128)��2S�󴮿ڿ�ʼ��Ч
  	 init_time += 1;
  	 if (init_time <= 4){
  	    DataGet = 128;
  	 }


  	 //����ת��1����0~1023��-511~512
     target = (ch[0] - 511)*(-2);

  	 // ����ת��3
  	 inData = DataGet * 4 - 512;

  	 // 0.5Sһ�μ���
     u = inData - target;
     y = u - 0.6 * u0 + 0.2 * y0;
     y0 = y;
     u0 = u;

     //����ת��2����-511~512��0~2047��
     target = y;   // ������
     target = (target + 511) * 2;

     //���Ʒ���
     if(target > 2047){
         target = 2047;
     }
     if(target < 0){
         target = 0;
     }

	 //�ش����Ҳֻ�ܵ�2047��������Ҫ������ֵ������
	 DataSend = target / 8;
   }


  // ��Ƭ���򴮿ڷ�����ֵ
  if(transMark == 1)// �������ڵ���־
  {
	 transMark = 0;
	 while(!(UCSR0A & (1<<UDRE0)));  //�жϴ��ڷ��ͼĴ����Ƿ�æ
	 uart_Putchar((char)DataSend);
  }


//  display(disValue);         // �����λ�������ʾAD�Ľ��
//  display_upper(dis_value);  // �ұ���λ�������ʾ������ֵ��0.5s����һ��

  display(DataGet);         // �����λ�������ʾ��Ƭ�����յ���ֵ
  display_upper(DataSend);   // �ұ���λ�������ʾ��Ƭ�����͸����ڵ�ֵ��������0.5S

 }

}