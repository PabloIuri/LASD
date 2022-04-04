/*
 * TacometroComVisorNokia5110.c
 *
 * Created: 06/02/2022 16:26:18
 * Author : Pablo
 * 
 * A distância é medida de acordo com o diâmetro da roda (2*pi*R)
 * A borda de descida da onda quadrada define a incrementação para verificar a distância percorrida
 * o período que entre uma borda de descida e outra definirá a velocidade do veículo
 
	calibragem: 4.1hz ,  246RPM,  30km/h D = 65cm 
			     41hz , 2460RPM, 300km/h D = 65cm
				 41hz , 2460RPM, 231km/h D = 50cm
				 
	KM/H = cm/us    * 3.6e4
	KM/H = cm/100us * 3.6e2
	
	Display nokia 84x48, letra pequena 7 pixels
 */ 

#define PI 3.141592654 
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1    // OBSERVAR EQUAÇÃO PARA O CALCULO DA TAXA DE TRANSMISSÃO

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>
//#include "nokia5110.h"
#include "SSD1306.h"
#include "Font5x8.h"




uint32_t tempo_100us               = 0;      // 4.294.967.295 * 100us = ~ 119.3 hrs
float    velocidadeVeiculo_kmph    = 0;      // 0-300
uint32_t distanciaPercorrida_cm    = 0;
float    RPM                       = 0; 
uint8_t  diametro_cm               = 0;
//float    distanciaPercorrida_km  = 0;      
uint16_t periodoRotacao_100us      = 0;
uint16_t leitura_ADC               = 0;
uint8_t  PINOS                     = 0x07;   // Variavel que será utilizada para armazenar estado dos pinos
uint32_t instante_borda_subida_PB0 = 0;
uint32_t tempo_delta               = 0;
char marcha                        = 'P';
//uint16_t  pedal                    = 0;
uint16_t  temperatura_bateria      = 35;
uint16_t  porcentagem_bateria      = 50;
uint16_t  sonar_cm_8               = 0;
unsigned char c_usart_recebido     = 'A';
uint8_t   temperatura_maxima_bat   = 30;
uint8_t   flag_send                = 0;
uint8_t   vigilante                = 0;


void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr >> 8);         // PARTE ALTA DO AJUSTE DA TAXA DE TRANSMISSÃO    (DESLOCANDO 8 BITS PARA DIREITA E LEITURA DOS PRIMEIROS 8 BITS) PARTE ALTA DE ubrr
	UBRR0L = (unsigned char)ubrr;                // PARTE BAIXA
	UCSR0B = (1 << TXCIE0)|(1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);  // HABILITA O TRANSMISSOR E O RECEPTOR
	UCSR0C = (1 << USBS0)|(3 << UCSZ00);         // AJUSTA O FORMATO DO FRAME : 8 BITS DE DADOS E 2 DE PARADA
}

void USART_Trasnmit(unsigned char data)
{
	while(!(UCSR0A & (1 << UDRE0)));             // ESPERA LIMPEZA DO REGISTRADOR DE TRANSMISSÃO
	UDR0 = data;                                 // COLOCA DADO NO REGISTRADOR E ENVIA
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1 << RXC0)));              // ESPERA O DADO SER RECEBIDO
	return UDR0;                                 // LE O DADO RECEBIDO E RETORNA
}

ISR(USART_RX_vect)
{
	c_usart_recebido = UDR0;
	
	if(c_usart_recebido == 'l')                            // LIMPAR TEMPERATURA MÁXIMA DA BATERIA
	{
		eeprom_write_byte((uint8_t*)5, 0x00);
		   
	}
	if(c_usart_recebido == 'v')
	{
		vigilante = 0xFF;
		PORTC |= 0b00001000;
	}
	if(c_usart_recebido == 'f')
	{
		vigilante = 0;
		PORTC &= 0b11110111;
	}
}

ISR(USART_TX_vect)
{
	flag_send = 1;
}

ISR(TIMER1_CAPT_vect)                            // Interrupção por captura do valor de TC1
{
	if(TCCR1B & (1 << ICES1))					 // Lê o valor da contagem do TC1 na borda de subida           
		instante_borda_subida_PB0 = ICR1;        // salva instante da subida 16bits
	else                                         // Lê o valor da contagem na borda de descida do sinal
		tempo_delta = (ICR1 - instante_borda_subida_PB0) * 4; // cada incremento de TC1 corresponde a 0.5us
	TCCR1B ^= (1 << ICES1);                      // inverte a borda de captura
}

ISR(TIMER2_COMPA_vect)                           //Interrupção do TC0 a cada 100us = (8 * (199 + 1)) / 16MHz
{
	tempo_100us++;
	// PORTC ^= 0b1000000;                          // Inverte estado de PB7 a cada 200us -> 5khz
}

ISR(INT0_vect)                                   // Interrupção PD2 sensor hall roda Calcula periodo de rotação da roda em 100us a cada descida do sensor hall
{												   
	static uint32_t instante_100us_anterior = 0;
	uint32_t  instante_100us = tempo_100us;
	
	periodoRotacao_100us = instante_100us - instante_100us_anterior;
	
	// Realiza o cálculo da distância percorrida a cada giro da roda
		
	distanciaPercorrida_cm += diametro_cm * PI; // fazer a cada rotação
	//distanciaPercorrida_km = distanciaPercorrida_cm / 100000;
		
	instante_100us_anterior = instante_100us;    	
}

ISR(ADC_vect)                                    // Interrupção contínua do ADC
{
	leitura_ADC = ADC;
}

ISR(PCINT2_vect)                                 // Interrupção por mudança de pino na porta D, LÊ BOTÕES E SALVA EEPROM
{
	if((PIND & 0b00010000) == 0)                 // BOTÃO '+' PD4
	{
		if(diametro_cm < 200)
		{
			diametro_cm++;
			eeprom_update_byte((uint8_t*)0, diametro_cm);    // ATUALIZA EEPROM
		}
		
	}
	if((PIND & 0b00100000) == 0)                 // BOTÃO '-' PD5
	{
		if(diametro_cm > 1)
		{
			diametro_cm--;
			eeprom_update_byte((uint8_t*)0, diametro_cm);    // ATUALIZA EEPROM
		}
		
	}
	if((PIND & 0b00001000) == 0)                 // MARCHA DRIVE PD3
	{
		marcha = 'D';
	}
	else                                         // MARCHA REVERSE
	{
		marcha = 'R';
	}
	if((PIND & 0b10000000) == 0)                 // MARCHA PARK PD7
	{
		marcha = 'P'; 
	}
// 	if((~PIND & 0b00000001) == 0)                // 0bxxxxxxx0 & 0b00000001 == 0
// 	{
// 		PORTD ^= 0b00000010;
// 		
// 		if (vigilante == 0xFF)                    // ALTERNA MODO VIGILANTE
// 		{
// 			PORTD &= 0b11111101;
// 			vigilante = 0;
// 		}
// 		else
// 		{
// 			PORTD |= 0b00000010;
// 			vigilante = 0xFF;                     
// 		}
// 		
		
	
}


void visorBCD(uint16_t v_atual)                  // Função que exibe velocidadeVeiculo_kmph no BCD a cada momento que é chamada
{
	static uint8_t pos = 0;                      // variavel do display 7seg que será exibido no momento
		
	switch (pos)
	{
		case 0: // 001 unidades
		  //PORTB =  0b1100000x;                   // Zera saída PB1..PB4, seta 0 no PB5 7seg de unidades 100
			PORTB &= 0b11000001;                   // PB0 inalterado
			PORTB |= 0b11000000;
			uint8_t num = ((v_atual/1)%10 << 1) & 0b00011110;
			PORTB |= num;   
		break;
		case 1: // 010 dezenas
		  //PORTB =  0b1010000x;                 // Zera saída PB1..PB4, seta 0 no PB6 7seg de dezenas (010)
			PORTB &= 0b10100001;                 // PB0 inalterado
			PORTB |= 0b10100000;
			PORTB |= (((v_atual/10)%10)<<1);
		break;
		case 2: // 100 centenas
		  //PORTB =  0b0110000x;                 // Zera saída PB1..PB4, seta 0 no PB7 7seg de centenas (110)
			PORTB &= 0b01100001;                 // PB0 inalterado
			PORTB |= 0b01100000;
			PORTB |= (((v_atual/100)%10)<<1);
		break;		
	}
	//0,1,2->
	pos == 2 ? pos = 0 : pos++;                  // incrementa ou zera contador 0..3
	
} 

void calculosTelemetria()
{
	// velocidade do veiculo kmph
	
	velocidadeVeiculo_kmph = diametro_cm * PI * 360;
	velocidadeVeiculo_kmph = velocidadeVeiculo_kmph / periodoRotacao_100us;
	 
	RPM = 600000 / periodoRotacao_100us;  
	
	static float distanciaPercorrida_cm_passado = 0;
	
	sonar_cm_8 = (int)tempo_delta/58;                                         // atualiza distancia sonar
	
	// Salvar na EEPROM 
	
	if (distanciaPercorrida_cm - distanciaPercorrida_cm_passado >= 100000.0)     // A cada 1km adicionado ao Hodômetro será salvo na EEPROM
	{
		while(!eeprom_is_ready());
		eeprom_update_float((float*)1, distanciaPercorrida_cm);
		distanciaPercorrida_cm_passado = distanciaPercorrida_cm;
	}
	
}


// protótipos
 
void leitura_sensores_ADC();
void visor_oled();

void envia2algarismos(void)
{
	if(c_usart_recebido == 'd')
	{
		UDR0 = (unsigned char)((temperatura_maxima_bat/10)%10);        // DEZENAS PRIMEIRO
	}
	
	if(flag_send)   // FLAG DA ULTIMA TRANSFERENCIA CONCLUÍDA
	{
		UDR0 = (unsigned char)((temperatura_maxima_bat/1)%10);     // ENVIA UNIDADES
		flag_send=0;
	}
}

void _6_ms(uint32_t instante_100us)              // 167hz taxa de atualização
{
	static uint32_t instante_100us_anterior = 0;
	
	if ((instante_100us - instante_100us_anterior) >= 60) // 60 * 100u = 0.006 
	{
		//Tarefa
		visorBCD((int)velocidadeVeiculo_kmph);                  
		//PORTB ^= 0b10000000;
		instante_100us_anterior = instante_100us;
	}
}

void _100_ms(uint32_t instante_100us)           // 10hz taxa de atualização
{
	static uint32_t instante_100us_anterior = 0;
	
	if ((instante_100us - instante_100us_anterior) >= 1000)   
	{
		// Funções executadas em 10hz
		calculosTelemetria();
		leitura_sensores_ADC();                              
		envia2algarismos();
		instante_100us_anterior = instante_100us;
	}
}

void _500_ms(uint32_t instante_100us)
{
	static uint32_t instante_100_us_anterior_500 = 0;
	
	if ((instante_100us - instante_100_us_anterior_500) >= 5000)
	{
		visor_oled();
	}
	
	instante_100_us_anterior_500 =  instante_100us;
}

int main(void)
{
	// Registradores de GPIO
	
	DDRB   =  0b11111110;   // Habilita saída PB1..PB7  3 X BCD + Comutador PB0 INPUT
	//DDRC   =  0b11011111;   // Habilita saída PC0..PC4  Display Nokia, entrada ADC PC5
	DDRD   =  0b01000010;   // PD2(SENSOR HALL), PD3(D/R), PD4(+), PD5(-), PD6(PWM), PD7(P) PD0(VIG)
	PORTD  =  0b00110000;   // Habilita pullups PD4(+) PD5(-)  
	PORTB  =  0b00000000;   // Desabilita pullups
	DDRC  |=  0b00001000;   // PC3 SAÍDA
	
	// Configuração do Timer T0  USADO COMO PWM   | Pinos PWM 0CR0A , 0CR0B = PD6, PD5 |
	
// 	TCCR0A = 0b00000010;  // Habilita modo CTC do TC0
// 	TCCR0B = 0b00000010;  // WGM02 da ultima tabela 0, prescale (clock/8)   // NOTA, SIMULIDE NÃO FUNCIONOU SEM PRESCALE
// 	OCR0A  = 199;         // Ajusta o comparador para o TC0 contar até 199 -> 100u
// 	TIMSK0 = 0b00000010;  // Habilita a interrupção na igualdade de comparação com 0CR0A.
 						  // A interrupção ocore a cada 100us = (8*(199+1))/16MHz
	
	// Configuração do Timer T1     | Pinos PWM 0CR1A , 0CR1B = PB1, PB2 | | Captura no PB0 ICP1 |
	
	TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10); // Habilita borda de subida, 010 -> Prescale 8 -> 8*(2^16)/16MHz
	TIMSK1 = 1 << ICIE1;                 // Habilita a interrupção por captura

	// Configuração do Timer T2     | Pinos PWM 0CR2A , 0CR2B = PB3, PD3 |

	TCCR2A = 0b00000010;  // Habilita modo CTC do TC2
	TCCR2B = 0b00000010;  // WGM02 da ultima tabela 0, prescale (clock/8)   // NOTA, SIMULIDE NÃO FUNCIONOU SEM PRESCALE
	OCR2A  = 199;         // Ajusta o comparador para o TC2 contar até 199 -> 100u  , (8*(199+1))/16MHz
	TIMSK2 = 0b00000010;  // Habilita a interrupção na igualdade de comparação com 0CR2A.

	
	// Configura interrupção para borda de descida para PD2, PD3
	
	EICRA  = 0b00000010; // 1010 borda de descida em PD2, PD3   
	EIMSK  = 0b00000001; // Habilita INT1, INT0
	PCICR  = 0b00000100; // HABILITA INTERRUPT PIN CHANGE 2 (PORTA D)
	PCMSK2 = 0b10111000; // HABILITA INTERRUPT PIN CHANGE PD3 PD4 PD5 PD7
	
	// Configuração ADC  (Desativar pullup do pino que será utilizado e configurár-lo como entrada)
	
	ADMUX  =  0b01000000; // Tensão de referência canal 0
	ADCSRA =  0b11100111; // b7 ADC Enable, b6 ADC start, b5 ADC auto trigger, b4 !Interrupt flag b3 ADC !Interrupt Enable, b0..b2 prescaler /128 para 10bits
	ADCSRB =  0b00000000; // Modo de conversão contínua
	DIDR0  =  0b00100000; // entradas ADC
	
	// Configuração do Timer T0 PWM na porta OCRA0, OCRB0
	
	TCCR0A = 0b10000011; // FAST PWM, TOP=0xFF, OC0A PD6 , PWM Não invertido 
	TCCR0B = 0b00000011; // LIGA TC0, prescaler = 64, fpwm = foscilador/(256 * prescaler) = 976hz
	//TCCR0A = 0b00100011; // FAST PWM NÃO INVERTIDO NO PINO 0CnB
	//TCCR0B = 0b00000100; // LIGA TC2 PRESCALER 64
	
	OCR0A  = 128;          // 50%  
	//OCR0B  = 0;          // Conf inicial zerada  
	
	// Habilita o flag geral de interrupções
	
	sei();
	
	// Carregar dados da EEPROM
	
	while(!eeprom_is_ready());
	diametro_cm = eeprom_read_byte((uint8_t*)0);
	while(!eeprom_is_ready());
	distanciaPercorrida_cm = eeprom_read_float((float*)1);
	while(!eeprom_is_ready());
	c_usart_recebido = (unsigned char)eeprom_read_byte((uint8_t*)5);	
	
	
	// Inicialização display Nokia
	
	//nokia_lcd_init();
	
	GLCD_Setup();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_InvertScreen();
	
	// Inicialização USART
	
	USART_Init(MYUBRR);
		
	while (1) 
    {   
		_6_ms(tempo_100us);   // atualiza 7seg BCD
		_100_ms(tempo_100us); // atualiza display nokia, cálculos sem prioridade, botões, ADC 
		_500_ms(tempo_100us); // atualiza display oled
	}
}


void visor_oled()
{
	char dis_km_str[4];
	char rpm[6];
	//rpm[5] = '\0';                              
	char dia_cm[4];
	char adc[4];
	char sonar_cm[4];
	sonar_cm[5] = '\0';
	char v_bateria[4];
	char t_bateria[4];
	
	
	sprintf(dis_km_str, "%u", (int)(distanciaPercorrida_cm/100000));
	sprintf(rpm, "%u", (int)RPM); //RPM
	sprintf(dia_cm, "%u", diametro_cm);
	sprintf(adc, "%u", leitura_ADC);
	sprintf(sonar_cm, "%u", sonar_cm_8);
	
	sprintf(v_bateria, "%u", porcentagem_bateria);
	sprintf(t_bateria, "%u", temperatura_bateria);
	
	GLCD_Clear();
	
	GLCD_GotoXY(1,1);
	GLCD_PrintString("LASD Car");
	GLCD_DrawLine(1, 10, 50, 10, GLCD_Black);
	
	if ( vigilante == 0xFF)
	{
		GLCD_GotoXY(80, 5);
		GLCD_PrintChar('V');                                        // MODO VIGILANTE
		GLCD_DrawRectangle(73, 1, 90, 15, GLCD_Black);              // RETANGULO VIGILANTE
			
	}
	
	GLCD_GotoXY(100, 15);
	GLCD_PrintString(t_bateria);
	GLCD_PrintString(" C");
	
	GLCD_GotoXY(100, 3);
	GLCD_PrintString(v_bateria);
	GLCD_PrintString(" %");
	
	GLCD_DrawRectangle(98, 1, 126, 25, GLCD_Black);            // RETANGULO BATERIA + TEMPERATURA
	
	GLCD_GotoXY(1, 36);
	GLCD_PrintString("D. Pneu: ");
	GLCD_PrintString(dia_cm);
	GLCD_PrintString(" cm");
	
	GLCD_GotoXY(1, 26);
	GLCD_PrintString("Sonar: ");
	GLCD_PrintString(sonar_cm);
	GLCD_PrintString(" cm");
	
	GLCD_GotoXY(1, 16);
	GLCD_PrintString(rpm);
	GLCD_PrintString(" rpm");
	
	GLCD_GotoXY(20, 52);
	GLCD_PrintString(dis_km_str);
	GLCD_PrintString(" km");
	GLCD_DrawRectangle(10, 48, 70, 62, GLCD_Black);          // RETANGULO DISTANCIA
	
	GLCD_GotoXY(115, 52);
	GLCD_PrintChar(marcha);
	GLCD_DrawRectangle(108, 48, 125, 62, GLCD_Black);        // RETANGULO MARCHA   17x20  xy
	 
	
	
	GLCD_Render();
		
}

void leitura_sensores_ADC()
{
	static uint8_t cont_canal = 0;
	
	switch(cont_canal)
	{
		case 0:  //LEITURA CANAL 0 - PEDAL ACELERADOR
			//pedal = leitura_ADC;
			if (sonar_cm_8 > 300)       // rever
			{
				// PWM CURSO PEDAL /4;
				OCR0A = ADC/4;
			}
			else if(velocidadeVeiculo_kmph > 20)
			{
				// PWM LIMITA EM 25;
				OCR0A = 25;
			}
			
			ADMUX = 0b01000001;        // MUDAR PARA CANAL 1
			
			break;
		case 1:  // LEITURA CANAL 1 - TENSÃO BATERIA
			porcentagem_bateria = ((uint32_t)ADC*100)/1023;
			
			ADMUX = 0b01000010;        // MUDAR PARA CANAL 2
			
			break;
		case 2:
			temperatura_bateria = (((uint32_t)ADC - 511)*122)/100;
			
			if(temperatura_bateria > temperatura_maxima_bat)             // ATUALIZA TEMPERATURA MAXIMA DA BATERIA
			{
				temperatura_maxima_bat = temperatura_bateria;
				eeprom_update_byte((uint8_t*)5, temperatura_maxima_bat);   // ATUALIZA NA EEPROM			
			}
			
			ADMUX = 0b01000000;        // MUDAR PARA CANAL 0
			
			break;
			
	}
	cont_canal == 2 ? cont_canal = 0 : cont_canal++;                  // incrementa ou zera contador 0..2
}
