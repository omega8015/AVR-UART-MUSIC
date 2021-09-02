#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define START_TC1 TCCR1B |= (1<<CS10)   //   ������ ������������ �� ������ TC1 ��� ������������
#define STOP_TC1  TCCR1B &= ~(1<<CS10); TCNT1H=0; TCNT1L=0;  //    H
#define DP 8000   //   ����� � �������� ����� ������ ��� ����� ��������
//#define F_CPU 16000000
#define LED 5
#define LED_ON PORTB |= (1<<LED)
#define LED_OFF PORTB &= ~(1<<LED)
#define MAX_TRACK_SIZE 60
#define SPLITTER ';'
#define ALL_IN_HIGH PORTC |= ((1<<DDC4) | (1<<DDC5)); PORTD |= (1<<DDD2);
#define COLUMN0_LOW PORTC &= ~(1<<DDC4)
#define COLUMN1_LOW PORTC &= ~(1<<DDC5)
#define COLUMN2_LOW PORTD &= ~(1<<DDD2)




// ������������ ������� ��1 ��� ��������� ��� �� �� ����� ������ �� �� 3 ������. 0 - ��� �����:
unsigned int notes [33] = {
45814,	43243,	40816,	38647,
36364,	34323,	32394,	30578,
28862,	27242,	25713,	24270,
22908,	21622,	20408,	19263,
18182,	17161,	16198,	15289,
14431,	13261,	12856,	12135,
11454,	10811,	10204,	9632,
9091,	8581,	8099,	7645,
0
};
enum names {FM, FMd, GM, GMd, AM, AMd, BM, C1, C1d, D1, D1d, E1, F1, F1d, G1, G1d, A1, A1d, B1,
			C2, C2d, D2, D2d, E2, F2, F2d, G2, G2d, A2, A2d, B2, C3, PS};
//----------------------------------------------------------------------------------------------


// �������� �������� ��� ��� ������ ������������:
unsigned long int timer_param [7] = 
			{50000, 100000, 200000, 300000, 400000, 600000, 1200000};
enum length {sxt, oct, qrt, qrtp, hlf, hlfp, full};
// ----------------------------------------------

unsigned char CDRW [10][MAX_TRACK_SIZE];   //   �������-���� � ����������
unsigned char track_buffer [MAX_TRACK_SIZE];   //   ���� �����

unsigned char rx_Byte;  //  �������������� 32 ���� - �����
unsigned char tx_Byte;

unsigned char song [2] [15] =  {
								{F1, F1, G1d, PS, F1, C1, D1d, F1},
								{sxt, sxt, sxt, sxt, sxt, sxt, oct, qrtp}
								};
unsigned char counter = 0;	//	������� ������ ��� ����� �����
unsigned char track_recieve_complete = 0;	//	����� ����� ��������
unsigned char buffer_overflow = 0;	//	������� ������������ �������� ������, ������� ������� ����


volatile unsigned long int t1,t2;   //	���������� ����� ��� ������� ������������� ������ � ����.

void init_TC1_Fast_PWM (void);		//	TC1 � Fast PWM mode
void init_keyboard (void);			//	������������� ������ ��� ����������
unsigned char key_scan (void);		//	����� ������� ������ ���������� 3�4
void play_note (unsigned char n, unsigned char t, unsigned char st);   //   n - ����� ����, t - ������������ �������� ����, ������� ��������
void initUART(long int baudrate);
void USART_Transmit(unsigned char chr);
void clear_track_buffer (void);

void show_track_buffer (void);
void show_track (unsigned char n);

unsigned char save_track (void);			//	�������� ��������������� ������� � ���������� �� "����"
unsigned char check_splitters_track (void);	//	�������� �� ������� ������������ � ������������� ������ +
unsigned char check_num_track (void);		//	�������� ������ ����� +
unsigned char check_name_track (void);		//	�������� �������� ����� +
unsigned char check_temp_track (void);		//	�������� ����� +
unsigned char check_lgst_track (void);		//	������ ��� ��������
unsigned char check_notes_track (void);		//	�������� ���
unsigned char find_end_track (void);		//	����� ������� ����� �����
void input_error (unsigned char e);			//	�������� ��� ������ � �������







int main(void)   // ********  ����������� ������������� ��������� ********

{
DDRB |= 1 << DDB5;

init_TC1_Fast_PWM();   //   ����� ������������� ������� TC1 � Fast PWM mode
clear_track_buffer ();
initUART(9600);

sei();   //   ���������� ���������� ����������




//test_recieve ();
//STOP_TC1;
while (1)
{
	if (track_recieve_complete)
	{
	LED_ON;
	save_track ();
	LED_OFF;
	}
	else
	{
	LED_ON;
	LED_ON;
	LED_ON;
	LED_OFF;
	}
}

}

//=================================================================================================================
void init_TC1_Fast_PWM (void)    //   TC1 � Fast PWM mode
{
DDRB |= 1 << DDB1;   //   ���� B, 1 ������ �������� �� �����
TCCR1A |= (1 << COM1A0) | (1<<WGM11) | (1<<WGM10);  //   Toggle OC1A/OC1B on Compare Match
TCCR1B |= (1 << WGM12) | (1<< WGM13);   			//   WGM10-13 � �������
}

void play_note (unsigned char n, unsigned char t, unsigned char st)    //   n - ����� ����, t - ������������ �������� ����, ������� ��������
{
	if (notes[n])    //  ���� �� 0 (�� �����), ������ ����������� ������� � ������� ��������� ������� � �������� ������.
	{
	OCR1AH = (notes[n] >> 8);
	OCR1AL = notes[n];
	START_TC1;
	}
	for (t1=0; t1<timer_param[t]; t1++);
	STOP_TC1;
	if (st)
	for (t2=0; t2<DP; t2++);
}

/* Init RX and TX  USART 9600/8-N-1*/

void initUART(long int baudrate)
{
	unsigned int bauddivider;
	bauddivider = (F_CPU/(16*baudrate)-1);  // Calculated divider
	UBRR0L = (bauddivider & 0xFF);  // Low byte
	UBRR0H = (bauddivider>>8);  // High byte
	UCSR0A = 0; // Clear flags
	UCSR0B = 1<<RXEN0|1<<TXEN0|1<<RXCIE0|0<<TXCIE0; // Enable RX and TX, Enable interrupts RX and TX
	
}


ISR (USART_RX_vect)		//	���������� �� ������ ����� UART
{
rx_Byte = UDR0;

if (track_recieve_complete)		//	��� ������� � ������ ��������������� ����� - ������� �� ��������� ����������
{
return;
}

if (buffer_overflow)
{
return;
}

if (counter > (MAX_TRACK_SIZE-1) )
{
counter = 0;
buffer_overflow = 1;
return;
}

track_buffer [counter] = rx_Byte;

if (rx_Byte == 0xFF)		//	�������� �� FF (����� �����)
{
track_recieve_complete = 1;  // ��������� ����� ���������� ������ �����
counter = 0;	//	�������� ������� ���� ��� ����� ���������� �����
return;
}
counter++;
}


void USART_Transmit( unsigned char chr )   // Send char
{
   /* Wait for empty transmit buffer */
   while ( !( UCSR0A & (1<<UDRE0)) );
      
   /* Put data into buffer, sends the data */
   UDR0 = chr;
}

void clear_track_buffer (void)
{
	for (unsigned char i=0; i<MAX_TRACK_SIZE; i++)
	{
	track_buffer [i] = 0;
	}
}

void show_track (unsigned char n)
{
unsigned char x;
unsigned char s = 0;

while (CDRW[n][s] != 255)
{
s++;
}
x = s;
	for (unsigned char i=0; i<=x; i++)
		{
		USART_Transmit (CDRW [n][i]);
		}
}

void show_track_buffer (void)
{
unsigned char x;
x = find_end_track();
for (unsigned char i=0; i<=x; i++)
	{
	USART_Transmit (track_buffer [i]);
	}
}

void input_error (unsigned char e)
{
	show_track_buffer ();		//	� ����� �� ���������
	USART_Transmit (e);
	clear_track_buffer ();
	counter = 0;
	track_recieve_complete = 0;
}

unsigned char chek_splitters_track (void)
{
if( (track_buffer[2]==SPLITTER) && (track_buffer[11]==SPLITTER) && (track_buffer[15]==SPLITTER) && (track_buffer[18]==SPLITTER) )
{
return 1;
}
else
{
input_error (SPLITTER);
return 0;
}
}

unsigned char check_num_track (void)
{
	if (track_buffer[0] == '#')
	{
		if ( (47<track_buffer[1]) && (track_buffer[1]<58) )
		{
		return 1;
		}
		else
		{
		input_error ('1');
		return 0;
		}
	}
	else
	{
	input_error ('0');
	return 0;
	}
}

unsigned char check_name_track (void)
{
unsigned char err = 0;		//	������� ������
for (unsigned char i=3; i<11; i++)
{
	if ( !( (33<track_buffer[i]) && (track_buffer[i]<126) ) )
	{
	err++;
	input_error ('N');
	return 0;
	}
}
return !err;
}

unsigned char find_end_track (void)		//	���������� ������� ����� ����� �����
{
unsigned char i = 0;
while (track_buffer[i] != 255)
{
i++;
}
return i;
}

unsigned char check_temp_track (void)
{
if ( (track_buffer[12]=='A') && (track_buffer[13]=='D') && (track_buffer[14]=='G') )	//	ADG - ������
{
return 1;
}
if ( (track_buffer[12]=='A') && (track_buffer[13]=='N') && (track_buffer[14]=='D') )	//	AND - �������
{
return 1;
}
if ( (track_buffer[12]=='M') && (track_buffer[13]=='D') && (track_buffer[14]=='R') )	//	MDR - ��������
{
return 1;
}
if ( (track_buffer[12]=='A') && (track_buffer[13]=='L') && (track_buffer[14]=='L') )	//	ALL - �������
{
return 1;
}
if ( (track_buffer[12]=='P') && (track_buffer[13]=='R') && (track_buffer[14]=='S') )	//	PRS - ������
{
return 1;
}
input_error ('T');
return 0;
}

unsigned char check_lgst_track (void)		//	���������� union ��� �������������� ���� ���� �� ������������������ � ���� �����
{											//	��������� ��� ����� � ������ ch_lgst, ����� �������� ������ ����� full_lgst
union char_and_int {						//	����� �� ���� �� ������, ��� � ������ �� ���� ����, ���������� ����� ���� ��������
	unsigned char ch_lgst [2];				//	���������� ���������� ���� ����� ����� (������� � ������� ����� �����)
	unsigned int full_lgst;
	} lgst;
lgst.ch_lgst[0] = track_buffer[16];
lgst.ch_lgst[1] = track_buffer[17];

switch (lgst.full_lgst)
{
	case 'L'+('G'<<8):						//	��� ����� G ���������� �� ������� ���� ������ ����� full_lgst, �������
	return 1;								//	��� ���������� ������ ����� ��� ����� G ���������� �� 256 (��������� ����� �� 8 ��������)
	break;									//	� ���������� ������ ������������ ��� ����� L, ������� ��������� �� ������ �������� �����

	case 'S'+('T'<<8):
	return 1;
	break;

	default:
	input_error ('.');
	return 0;
}
}

unsigned char check_notes_track (void)
{
unsigned char x;	//		���������� ��� find end track
x = find_end_track();
	if (track_buffer[19] == 255)	//	 �������� �� ������� ��� ������
	{
	input_error ('F');
	return 0;
	}
	for (unsigned char i=19; i<x; i+=2)
	{
		if ( (32<track_buffer[i]) || (track_buffer[i]<0) )
		{
		input_error ('t');
		return 0;
		}
		if ( (6<track_buffer[i+1]) || (track_buffer[i+1]<0) )
		{
		input_error ('L');
		input_error (track_buffer[i+1]);
		return 0;
		}
	}
return 1;
}

unsigned char save_track (void)
{
unsigned char x;
if ( chek_splitters_track() && check_num_track () && check_name_track () && check_temp_track () && check_lgst_track () && check_notes_track () )
{
	x = find_end_track ();
	for (unsigned char i=0; i<MAX_TRACK_SIZE; i++)		//	����������� ��������� ����� � ���������� (������ 10�60)
		{
		CDRW[(track_buffer[1]-48)][i] = track_buffer[i];
		}
	for (unsigned char i=19; i<=x; i+=2)				//	��� �������; ���������� �������������� �������
		{
		play_note (CDRW[(track_buffer[1]-48)][i], CDRW[1][i+1], 1);
		}
	clear_track_buffer ();
	track_recieve_complete = 0;
}
else
{
	USART_Transmit ('Q');
	return 0;
}
return 1;
}


void init_keyboard (void)
{
PORTC |= 0x0F;				//	���� C ����� ���������� ������ �������� �� ����. ������ � ������� ������� ����� C ��������
							//	�������� ���������� ������������� ���������.
DDRC |= ((1<<DDC4) | (1<<DDC5));	//	4 � 5 ������� ����� C ��������� �� �����
PORTC |= ((1<<DDC4) | (1<<DDC5));
DDRD |= (1<<DDD2);					//	2 ������ ����� D �������� �� �����
PORTD |= (1<<DDD2);
}


/*#define ALL_IN_HIGH PORTC |= ((1<<DDC4) | (1<<DDC5)); PORTD |= (1<<DDD2);
#define COLUMN0_LOW PORTC &= ~(1<<DDC4)
#define COLUMN1_LOW PORTC &= ~(1<<DDC5)
#define COLUMN2_LOW PORTD &= ~(1<<DDD2)*/
unsigned char key_scan (void)
{
ALL_IN_HIGH;
COLUMN0_LOW;
COLUMN1_LOW;
COLUMN2_LOW;


}
