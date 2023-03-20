#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

// Servo controller with I2C interface.
// Generates PWM signals for up to 10 servos.

// Written for ATtiny26, using internal 4Mhz RC oscillator

//* Original code by Stefan Frings
//* Released under GPL
//* Additional comments (marked with '//*' or '/**'), corrections, and bug fixes by ftjuh
//* note: ATtiny26 bugfixes are not tested, I don't own one

// 1  PB0 SDA
// 2  PB1 activity LED (low active)
// 3  PB2 SCL
// 4  PB3 i2c address bit 0
// 5  VCC 3.3 - 5.5V
// 6  GND
// 7  PB4 i2c address bit 1
// 8  PB5 servo 9
// 9  PB6 servo 8
// 10 /Reset optional
// 11 PA7 servo 7
// 12 PA6 servo 6
// 13 PA5 servo 5
// 14 PA4 servo 4
// 15 VCC 3.3 - 5.5V
// 16 GND
// 17 PA3 servo 3
// 18 PA2 servo 2
// 19 PA1 servo 1
// 20 PA0 servo 0

#define SDA_mask (1 << 0) //* PB0
#define SCL_mask (1 << 2) //* PB2

// The input pins for i2c address bits have internal pull-up's.
// The remaining address bits are defined here:
#define ADR_b2 1
#define ADR_b3 1
#define ADR_b4 0
#define ADR_b5 0
#define ADR_b6 0
// In addition to the configured i2c address, the device also accepts address 0000000.

// i2c protocol:
// Send START condition
// Send 0x1E (assuming you have all address inputs not connected)
// The device answers with ACK
// Send 1-10 data bytes
// The device answers each byte with ACK
// Send STOP condition

// The first data byte controls servo0, the second byte controls sero 1, ...+
// Normal values are in the range 62-125, for pulses of 1-2ms duration.
// The value 94 genereates pulses of 1.5ms, which is for the center position.
// The special value 0 switches the PWM signal off.
// Each Servo has different characteristics, so you need to try the exact 
// values yourself.

// Output timing diagram:
// Servo 0: _/~\_________________/~\_________________/~\_
// Servo 1: _____/~\_________________/~\_________________
// Servo 2: _________/~\_________________/~\_____________
// Servo 3: _____________/~\_________________/~\_________
// Servo 4: _________________/~\_________________/~\_____
// Servo 5: _/~\_________________/~\_________________/~\_
// Servo 6: _____/~\_________________/~\_________________
// Servo 7: _________/~\_________________/~\_____________
// Servo 8: _____________/~\_________________/~\_________
// Servo 9: _________________/~\_________________/~\_____

//                           |-------20ms--------|


// PWM Values for the servos
volatile uint8_t servo[10];

// Activity Led timeout with initial blink of 400ms
volatile uint8_t ledTimeout=100; // 100*4ms

// Timer 1 Compare A Interrupt
ISR(TIMER1_CMPA_vect) {    
    // Sets servo 0-4 pins to low
    PORTA &= ~1;
    PORTA &= ~2;
    PORTA &= ~4;
    PORTA &= ~8;
    PORTA &= ~16;
}

// Timer 1 Compare B Interrupt
ISR(TIMER1_CMPB_vect) {    
    // Sets servo 5-9 pins to low
    PORTA &= ~32;
    PORTA &= ~64;
    PORTA &= ~128;
    PORTB &= ~64;
    PORTB &= ~32;
}

// Timer 1 Overflow Interrupt 
ISR(TIMER1_OVF1_vect) {
    static uint8_t loop;

    // Set servo pins to high, round-robin, two pins per interrupt
    if (servo[loop] != 0) {
        switch (loop) {
            case 0: PORTA |= 1;  break;
            case 1: PORTA |= 2;  break;
            case 2: PORTA |= 4;  break;
            case 3: PORTA |= 8;  break;
            case 4: PORTA |= 16; break;
        }
    }
    if (servo[loop+5] != 0) {
        switch (loop) {
            case 0: PORTA |= 32;  break;
            case 1: PORTA |= 64;  break;
            case 2: PORTA |= 128; break;
            case 3: PORTB |= 64;  break;
            case 4: PORTB |= 32;  break;
        }
    }

    // Set timer compare values for the next (not the current) loop
    if (++loop>4) {
        loop=0;
    }
    OCR1A=servo[loop];
    OCR1B=servo[loop+5];

    // Switch activity led off when it's timeout has expired
    if (ledTimeout!=0) {
        ledTimeout--;
        if (ledTimeout==0) {
            PORTB |= 2;
        }
    }
}


int main() {
    // Configure pin direction and pull-ups
    DDRA  = 255;
    PORTA = 0;
    DDRB  = 2|32|64;
    PORTB = 8|16|32;
    
    // Timer 1: prescaler 64, Inteerupt on compare A, B and overflow
    TCCR1A = 0;
    TCCR1B = (1<<CS10) | (1<<CS11) | (1<<CS12);
    TIMSK  = (1<<OCIE1A) | (1<<OCIE1B) | (1<<TOIE1);

    // Enable interrupts
    sei();

    // Receive data from I2C in the main loop
    while (1) {

        // Enable driving SCL
        DDRB |= 4;
        PORTB |= 4;    
        // SDA=input
        DDRB &= ~1;    

        // Clear i2c status flags and wait for start condition
        USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
        USICR = (1<<USIWM1) | (1<<USICS1);
        while (!(USISR & (1<<USISIF)));
   
        // Wait until end of start condition or begin of stop condition 
        //* while ((PINB & 4) && !(PINB & 1));
        uint8_t PINB_tmp; //* needed for fixing bug 1
        do {
          PINB_tmp = PINB;
        } while ((PINB_tmp & SCL_mask) && !(PINB_tmp & SDA_mask));

        // Break when a stop condition has been received while waiting
        if ((PINB_tmp & SDA_mask) and (PINB_tmp & SCL_mask)) { //* fixes bug 1
          //* break;
          continue; //* fixes bug 2
        }
       
        // Clear i2c status flags and prepare to receive the device address
        USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
        USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);

        // Wait for byte received or stop condition
        while (!(USISR & ((1<<USIOIF) | (1<<USIPF))));

        // If byte received
        if ((USISR & (1<<USIOIF))) {

            // If the address is 0 or matches the configured address
            // All bits are shifted 1 to the left as required by the i2c protocol.
            uint8_t myAddress=(ADR_b6<<7) | (ADR_b5<<6) | (ADR_b4<<5) | (ADR_b3<<4)  | (ADR_b2<<3) | ((PINB & 24)>>2);
            if (USIDR==0 || (USIDR==myAddress)) {

                // Switch activity LED for 80ms on
                PORTB &= ~2;
                ledTimeout=20; // 20*4ms

                // Send ack bit
                USIDR=0;
                DDRB |= 1;
                USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | 14;
                while (!(USISR & (1<<USIOIF)));
                DDRB &= ~1;  

                // Receive up to 10 bytes
                for (uint8_t channel=0; channel<10; channel++) {

                    // Clear i2c status flags and prepare to receive next byte
                    USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
                    USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);

                    // Wait for byte received or stop condition
                    while (!(USISR & ((1<<USIOIF) | (1<<USIPF))));

                    // If byte received
                    if ((USISR & (1<<USIOIF))) {

                        // copy to the servo value array
                        servo[channel]=USIDR;

                        //Send ack bit
                        USIDR=0;
                        DDRB |= 1;
                        USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | 14;
                        while (!(USISR & (1<<USIOIF)));
                        DDRB &= ~1;
                    }

                    // else if stop condition received, break the for loop
                    else {
                       break;
                    }  
                }  
            }
        }    
    }
}
