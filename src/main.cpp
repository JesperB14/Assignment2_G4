//#define F_CPU 1000000UL //inte relaterad till clk_io -- se: https://stackoverflow.com/questions/55484116/why-is-define-f-cpu-have-no-effect-on-avr-code-delay-ms-function
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

const int pin13 = 13; // set to pin9
const int pin9 = 9; // set to pin9
const int pin10 = 10; // set to pn10 
const int pot_meter_pin = A0; // potentiometer <-> arduino input pin

// ----------- Teori----------------
// PWM capabilities 328p: https://www.electronicwings.com/users/sanketmallawat91/projects/215/frequency-changing-of-pwm-pins-of-arduino-uno

void setup() {
  pinMode(pin9, OUTPUT); // configure pin9 as output
  pinMode(pin10, OUTPUT); // configure pin10 as output
  pinMode(pin13,OUTPUT);
  pinMode(pot_meter_pin, INPUT); // configure potentiometer <-> arduino pin as input
  
  //Using fast-pwm on timer/counter1 (pins PB2/PB1 = D10,D9)
  // f_OCnxPWM = f_clk_io/(N(1+TOP))  se sida 102 i datablad 328p
  //Fast-pwm med ICR1 som TOP värde: WGM13, WGM12, WGM11, WGM10 = 0b1110 (se sida 109) i datablad 328p
  //counter prescaler N = 1 <===> CS12, CS11, CS10 = 0b001
  //COM-bitarna sätter beteendet hos OC1A och OC1B (D10,D9) benen. COM1A1,COM1A0,COM1B1,COM1B0 = 0b1111 sets pins to compare match (se sida 102 i datablad 328p)
  //CLKPR styr prescalern för system-klockan ===> CLKPS3, CLKPS2, CLKPS1, CLKPS0 = 0b0010 för prescaler = 4 ----> Fsys = 4MHz
  //Extern clocka på 16MHz (inte 328ps interna 8MHz clocka)
  
  //OBS! man måste stänga av interrupts och skriva till CLKPR registret på två rader för att kunna ändra systemklockans prescaler!
  cli();
  CLKPR = (1 << CLKPCE);
  CLKPR = (0 << CLKPS3) | (0 << CLKPS2) | (1 << CLKPS1) | (0<< CLKPS0);
  sei();
  TCCR1B =  (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10); 
  TCCR1A =  (1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (0 << WGM10);  
  ICR1 = 1999; //ICR1 registret består av två 8-bitars register (ICR1H och ICR1L)

  //Setting up timer0 for 20Hz interrupt call
  TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // 1024 clock prescaler
  TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (0 << WGM00); // WGMxx sets the mode of the timer
  TIMSK0 = (0 << OCIE0B) | (1 << OCIE0A) | (0 << TOIE0);  // enable the TIMER0 OVERFLOW INTERRUPT
  OCR0A = 96;  //Timer/Counter0 output compare A match interrupt enable (when TCNT0 reaches OCR0A, the TIMER0_COMP_vect is updated)

  //Setting up adc
  //Mux set to ADC0 (PC0/A0), reference voltage set to AVCC, ADLR = 1 ---> right adjusted result.
  ADMUX = ( 0 << REFS1 ) | ( 1 << REFS0 )| ( 0 << ADLAR ) | ( 0 << MUX3 ) | ( 0 << MUX2 ) | ( 0 << MUX1 ) | ( 0 << MUX0 ); 
  // ADEN (adc-enable), ADSC (adc start-conversion ---> write to one to start conv.)
  // ADATE (auto-trigger enable), ADIF (adc-interrupt flag), ADIE (adc-interrupt enable), ADPS (adc prescaler bits)
  // ADSC is reset to zero after every conversion!
  ADCSRA = ( 1 << ADEN ) | ( 0 << ADSC ) | ( 0 << ADATE ) | ( 0 << ADIF ) | ( 1 << ADIE ) | ( 0 << ADPS2 ) | ( 0 << ADPS1 ) | ( 1 << ADPS0 );
}

ISR(TIMER0_COMPA_vect){
    ADCSRA = ADCSRA | (1 << ADSC);
    PORTB ^= (1 << PB5); //så man kan proba d13 och kolla att ADCn körs i 20Hz
}

ISR(ADC_vect){
  if (ADC > 511){
    OCR1A = ICR1;
    OCR1B = ICR1-(ADC*0.001953-1)*ICR1;
  } else {
    OCR1A = (ADC*0.001957)*ICR1;
    OCR1B = ICR1;
  } 
}

void loop() {
  //¯\_(ツ)_/¯ Det här är ett test
}