#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define set_bit(reg, pin)    (reg |= (1<<pin))
#define clr_bit(reg, pin)    (reg &= ~(1<<pin))
#define toggle_bit(reg, pin) (reg ^= (1<<pin))
#define tst_bit(reg, pin)    (reg & (1<<pin))

#define DISPLAY1 PC5
#define DISPLAY2 PC4
#define DISPLAY3 PC3
#define DISPLAY4 PC2

#define NUM_SENSORS 2

void setup(void);
void loop(void);

void f_timers(void);
void read_keyb(void);
void mux_display(void);
uint16_t adc_read(void);
void adc_conversion_ch(uint8_t channel);
void adc_maq(void);
void cpwm(void);

uint8_t timer_pwm = 1, pwm_on = 0;
uint16_t display1 = 0, display2 = 0, display3 = 0, display4 = 0;
uint16_t AD[NUM_SENSORS] = {0}, pwm = 0;


uint8_t hex_numbers[] = 
{
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01100111, // 9
};

ISR(PCINT0_vect) //houve alteracao de nivel logico
{
  read_keyb();
}

ISR(ADC_vect)
{
  adc_maq();
}

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6;
  f_timers();
}

int main()
{
  setup();
  while(1) loop();
}

void setup(void)
{
  cli(); //desabilta a chave geral de interrupcao
  DDRD   = 0b11111111;  //PD2 e PD3 como entrada
  DDRB   = 0b11111110; 
  DDRC = 0b11111100; //PC0 e PC1 como entradas
  PORTD  = 0;  //inicia apagado
  PORTB  = 0b00100000;
  ADMUX  = 0b01000000; //referencia AVCC de 5V, justificado a direita nenhum canal selecionado (ADC0 selecionado por padrao)
  ADCSRA = 0b10101101; //AD habilitado, interrupcao habilitada, PS de 32 para ADC Clock de 250 kHz
  ADCSRB = 0x00; //modo livre
  TCCR0B = 0x03; // 0b00000011 //3
  TCNT0 = 6;
  TIMSK0 = 0x01; //habilita a interrupcao do timer 0
  TCCR1A = 0b10100011;
  TCCR1B = 0b00001010; //PWM fast mode de 10 bits comprescaler de 8
  adc_maq();
  sei(); //habilta a chave geral de interrupcao
}

void loop(void)
{
}

void f_timers(void)
{
  static uint16_t counter0 = 1, counter1 = 1, counter2 = 1, counter3 = 1;

  if(counter0 < 200)
  {
    counter0++;
  }

  else
  {

    display4 = AD[0]/1000;
    display3 = (AD[0]%1000)/100;
    display2 = (AD[0]%100)/10;
    display1 = AD[0]%10;
    
    counter0 = 1;
  }

  if(counter1 < 10)
  {
    counter1++;
  }

  else
  {
    mux_display();
    counter1 = 1;
  }

  if(counter2 < 10)
  {
    counter2++;
  }

  else
  {
    timer_pwm = AD[1] * 200 / 1023;
    counter2 = 1;
  }

  if (counter3 < timer_pwm) 
  {
    counter3++;
  }

  else 
  {
    read_keyb();
    if(pwm_on) cpwm();
    counter3 = 1;
  }

}

void read_keyb(void)
{
  static uint8_t memory_button1 = 1, button1 = 0;

  if(tst_bit(PINB, PB0))
  {
      button1 = 1;
  }

  else
  {
      button1 = 0;
  }

  if(button1 < memory_button1)
  {   
    pwm_on ^= 1;
    pwm = 0;
    OCR1A = 0;
  }

  memory_button1 = button1;
}

void mux_display(void)
{
  static uint8_t mux = 0;

    switch (mux)
    {
      case 0:
        PORTD = hex_numbers[display1];
        set_bit(PORTC, DISPLAY1);
        clr_bit(PORTC, DISPLAY4);
        mux = 1;
        break;
      
      case 1:
        PORTD = hex_numbers[display2];
        set_bit(PORTC, DISPLAY2);
        clr_bit(PORTC, DISPLAY1);
        mux = 2;
        break;

      case 2:
        PORTD = hex_numbers[display3];
        set_bit(PORTC, DISPLAY3);
        clr_bit(PORTC, DISPLAY2);
        mux = 3;
        break;

      case 3:
        PORTD = hex_numbers[display4];
        set_bit(PORTC, DISPLAY4);
        clr_bit(PORTC, DISPLAY3);
        mux = 0;
        break;
    }
}

uint16_t adc_read(void)
{
  unsigned int dado = (ADCH<<8) | ADCL;
  return dado;
}

void adc_conversion_ch(uint8_t channel)
{
  ADMUX &= 0xf0;
  ADMUX |= (channel & 0x0f);  
  ADCSRA |= (1<<ADSC);//inicio a conversao
}

void adc_maq(void)
{
  static uint8_t estado = 10;
  
  switch (estado) 
  {    
    case 0:
      estado = 1;
      AD[0] = adc_read();
      adc_conversion_ch(1);
      break;
        
    case 1:
      estado = 0;
      AD[1] = adc_read();
      adc_conversion_ch(0);
      break;

    default:
      estado = 0;
      AD[0] = adc_read();
      adc_conversion_ch(1);
      break; 
  }    
}

void cpwm(void) 
{
  if (pwm < 1023) 
  {
    pwm += 10;
  }
  else 
  {
    pwm = 1023;
  }

  OCR1A = pwm;
}