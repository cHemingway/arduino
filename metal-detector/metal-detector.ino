//AVR
#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>
#include <JeeLib.h> // Low power functions library

//bits clear
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
//bits set
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Arduino pins
//TODO: Add pin maps here

//constants
//------------ pins
const int genPin = 8; //gen pin
const int btnPin = 2; //btn pin
const int ain0Pin = 6; //AIN0 pin
const int ain1Pin = 7; //AIN1 pin
//------------ delays
const unsigned int PULSE_US = 250; //pulse length, us
const unsigned int DELAY_US = 5; //delay time, us
const unsigned int PAUSE_MS = 20;  //pause length, ms
const unsigned int BLINK_MS = 250; //blink length, ms

const int BLINK_TIMES = 3;//init blink times


const int PROTECT = 3;
const int AVG_COUNTS = 20;
const int ZERO_COUNTS = 4000;

//------------ btn

//vars
int count;
int i;
int zero = 0;
int counter;//counter
float count_avg;
int btnState = 0;
boolean zeroing = false;
unsigned int zero_count = 0;
int delta = 0;//diff

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

boolean target = false;
void setup() {
  Serial.begin(9600);
  pinMode(genPin, OUTPUT);//gen pin -> out
  pinMode(btnPin, INPUT_PULLUP); //btn pin <- in
  digitalWrite(btnPin, HIGH); //btn pin pull-up en

  pinMode(ain0Pin, INPUT);//AIN0 -> input
  pinMode(ain1Pin, INPUT);//AIN1 -> input
  ACSR = 0b00000000;
  ADCSRA = 0b10000000;
  ADCSRB = 0b00000000;
  //zeroing init
  zero_count = ZERO_COUNTS;
  zero = 0;
  zeroing = true;
  target = false;

  power_adc_disable();//ADC disable
  power_spi_disable();//SPI disable
  power_twi_disable();//Two Wire Interface disable
  power_usart0_disable();//USART0 disable
}

void loop() {
  while (true){
    noInterrupts();//interrupts disable
    count = 0;
    //pulse turned on
    PORTB |= 0b00000001; //1
    delayMicroseconds(PULSE_US);
    //pulse turned off
    PORTB &= ~0b00000001; //0
    //delay time
    delayMicroseconds(DELAY_US);
    //low wait
    Serial.println("Pulsed!");
    do
    {
    }

    //calculates time voltage is above certain threshold
    while ((ACSR & 0b00100000) != 0);
    //hi wait
    do
    {
      count=count+1;
    }
    while ((ACSR & 0b00100000) == 0);
    interrupts();//interrupts enable
    Serial.println("GOT count, interrupts ON");

    //zeroing
    if (zeroing) {
      Serial.println("Zeroing");
      if (count>zero) {
        zero = count;//new zero value
      }
      zero_count--;//zero counter decrement
      if (zero_count == 0) {
        zeroing = false;//zeroing O.K. 
        counter = 0;//average counter reset
        count_avg = 0;
      }
      else {
        //*********
        for (i = 0 ; i < ( ((float) ((float) (ZERO_COUNTS-zero_count)/ZERO_COUNTS)))*10 ; i++)
        {

        }

      }
    }
    else {
      delta = count - zero;
      //target check
      if (count > (zero+PROTECT)) {
        //target is found!!!
        Serial.println("Target Found!");
        target = true;
        //delta indication
        //???
      }
      else
      {
        //target not found
        Serial.println("Target not found");
        target = false;
      }

      //avg calculation
      count_avg = (count_avg*counter+count)/(++counter);
      if (counter == AVG_COUNTS) {
//        //avg display
        count_avg = 0;//avg reset
        counter = 0;//counter reset
            
      }
    }
    
    //pause between pulses
    if (target) {
        delay(PAUSE_MS);  
      }
      else {
        Sleepy::loseSomeTime(PAUSE_MS);
    }
    
    //zeroing pushbutton check
    btnState = digitalRead(btnPin);//btn reading
    if ((btnState == 0) && (zeroing == false)) {
      //button is pressed  
      zero_count = ZERO_COUNTS;
      zero = 0;
      zeroing = true;
      target = false;
    }

    //next pulse
  }
}
