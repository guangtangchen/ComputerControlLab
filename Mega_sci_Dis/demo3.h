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
unsigned int ADChannel=0;
unsigned int ch[4];
int key =0;
//œ‘ æ¬Î
unsigned char g_aDisplayBuf[16]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07, \
0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71}; //0 1 2 ...9 a b c d e f


void port_init(void);
void timer1_init(void);
void timer3_init(void);
void uart0_init(void);
void uart1_init(void);
void adc_init(void);
void init_devices(void);
void delay_1us(void);
void delay_us(unsigned int n);
void delay_ms(unsigned int n);
void uart_Putchar(char c);
void display(unsigned int value);
void display_v(unsigned int value);