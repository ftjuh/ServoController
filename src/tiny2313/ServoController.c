#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

// Servo controller with I2C interface.
// Generates PWM signals for up to 10 servos.

// Written for ATtiny2313, using internal 4Mhz RC oscillator

//* Original code by Stefan Frings
//* Released under GPL
//* Additional comments (marked with '//*' or '/**'), corrections, and bug fixes by ftjuh


//* ATtiny2313 physical pins, sorted by pin number

// 1 /Reset optional
// 2  PD0 servo 0
// 3  PD1 servo 1
// 4  PA1 servo 2
// 5  PA0 servo 3
// 6  PD2 servo 4
// 7  PD3 servo 5
// 8  PD4 servo 6
// 9  PD5 servo 7
// 10 GND
// 11 PD6 servo 8
// 12 PB0 servo 9
// 13 PB1 i2c address bit 0
// 14 PB2 i2c address bit 1
// 15 PB3 i2c address bit 2
// 16 PB4 i2c address bit 3
// 17 PB5 SDA
// 18 PB6 activity LED (low active)
// 19 PB7 SCL
// 20 VCC 3.3 - 5.5V


//* ATtiny2313 physical pins, sorted by port

//* 2  PD0 servo 0
//* 3  PD1 servo 1
//* 6  PD2 servo 4
//* 7  PD3 servo 5
//* 8  PD4 servo 6
//* 9  PD5 servo 7
//* 11 PD6 servo 8

//* 5  PA0 servo 3
//* 4  PA1 servo 2

//* 12 PB0 servo 9
//* 13 PB1 i2c address bit 0
//* 14 PB2 i2c address bit 1
//* 15 PB3 i2c address bit 2
//* 16 PB4 i2c address bit 3
//* 17 PB5 SDA
//* 18 PB6 activity LED (low active)
//* 19 PB7 SCL


// The input pins for i2c address bits have internal pull-ups.
// The remaining address bits are defined here:
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

// Timer 0 Compare A Interrupt
ISR(TIMER0_COMPA_vect) {    
    // Sets servo 0-4 pins to low 
    //* actually, only one of these is high at a single moment (see timing diagram),
    //* but it's easier and safe to just bring them all to low
    PORTD &= ~1; //* PD0 servo 0
    PORTD &= ~2; //* PD1 servo 1
    PORTA &= ~2; //* PA1 servo 2
    PORTA &= ~1; //* PA0 servo 3
    PORTD &= ~4; //* PD2 servo 4
}

// Timer 0 Compare B Interrupt
ISR(TIMER0_COMPB_vect) {    
    // Sets servo 5-9 pins to low
    //* actually, only one of these is high at a single moment (see timing diagram),
    //* but it's easier and safe to just bring them all to low
    PORTD &= ~8;  //* PD3 servo 5
    PORTD &= ~16; //* PD4 servo 6
    PORTD &= ~32; //* PD5 servo 7
    PORTD &= ~64; //* PD6 servo 8
    PORTB &= ~1;  //* PB0 servo 9
}

// Timer 0 Overflow Interrupt 
ISR(TIMER0_OVF_vect) {
    static uint8_t loop;

    // Set servo pins to high, round-robin, two pins per interrupt
    if (servo[loop] != 0) {
        switch (loop) {
            case 0: PORTD |= 1; break;
            case 1: PORTD |= 2; break;
            case 2: PORTA |= 2; break;
            case 3: PORTA |= 1; break;
            case 4: PORTD |= 4; break;
        }
    }
    if (servo[loop+5] != 0) {
        switch (loop) {
            case 0: PORTD |= 8;  break;
            case 1: PORTD |= 16; break;
            case 2: PORTD |= 32; break;
            case 3: PORTD |= 64; break;
            case 4: PORTB |= 1;  break;
        }
    }

    // Set timer compare values for the next (not the current) loop
    if (++loop>4) {
        loop=0;
    }
    OCR0A=servo[loop];   //* will trigger TIMER0_COMPA_vect
    OCR0B=servo[loop+5]; //* will trigger TIMER0_COMPB_vect

    // Switch activity led off when it's timeout has expired
    if (ledTimeout!=0) {
        ledTimeout--;
        if (ledTimeout==0) {
            PORTB |= 64;
        }
    }
}


int main() {
    // Configure pin direction and pull-ups
    //* PA0 servo 3
    //* PA1 servo 2
    DDRA  = 1|2; // outputs, all others inputs

    //* PB0 servo 9
    //* PB6 activity LED (low active)
    DDRB  = 1|64; // outputs, all others inputs

    //* PB1 i2c address bit 0
    //* PB2 i2c address bit 1
    //* PB3 i2c address bit 2
    //* PB4 i2c address bit 3
    PORTB = 2|4|8|16; // activate pullups

    //* PD0 servo 0
    //* PD1 servo 1
    //* PD2 servo 4
    //* PD3 servo 5
    //* PD4 servo 6
    //* PD5 servo 7
    //* PD6 servo 8
    DDRD  = 1|2|4|8|16|32|64; // outputs, all others inputs

    
    // Timer 0: fast PWM, prescaler 64, IRQ on overflow, compare match A and compare match B.
    TCCR0A = (1<<WGM01) | (1<<WGM00);
    TCCR0B = (1<<CS01)  | (1<<CS00);
    TIMSK  = (1<<TOIE0) | (1<<OCIE0A) | (1<<OCIE0B);

    // Enable interrupts
    sei();

    /**
     * The target/slave can insert wait states at start or end of transfer by 
     * forcing the SCL clock low. This means that the master must always check 
     * if the  SCL line was actually released after it has generated a positive edge.
     */
    
    // Receive data from I2C in the main loop
    while (1) {

        // Enable driving SCL
        DDRB |= 128;  //* SCL is output
        PORTB |= 128; //* SCL high
        // SDA=input
        DDRB &= ~32;

        // Clear i2c status flags and wait for start condition        
        
        USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
        /** 
         * USISR – USI Status Register - reset relevant flags by setting them to 1
         * 
         * USISIF: Start Condition Interrupt Flag --> is set (to one) when a 
         * start condition has been detected; The flag will only be cleared by 
         * writing a logical one to the USISIF bit.
         * 
         * USIOIF: Counter Overflow Interrupt Flag --> is set (one) when the 
         * 4-bit counter overflows (i.e., at the transition from 15 to 0);  The 
         * flag will only be cleared if a one is written to the USIOIF bit.
         * 
         * USIPF: Stop Condition Flag --> is set (one) when a stop condition has 
         * been detected. The flag is cleared by writing a one to this bit. 
         * 
         * USIDC: Data Output Collision --> is logical one when bit 7 in the 
         * USI Data Register differs from the physical pin value. The flag is 
         * only valid when two-wire mode is used. This signal is useful when 
         * implementing Two-wire bus master arbitration.
         */
        
        USICR = (1<<USIWM1) | (1<<USICS1);
        /** 
         * USICR – USI Control Register
         *
         * USIWM[1:0]: Wire Mode --> Two-wire mode. Uses SDA (DI) and SCL (USCK) pins
         * 
         * USICS[1:0]: Clock Source Select --> External, positive edge (USCK/SCL)
         */

        //* read USI status register and wait for start condition interrupt flag
        while (!(USISR & (1<<USISIF))); 

        // Wait until end of start condition or begin of stop condition 
        while ((PINB & 128) && !(PINB & 32));  //* while SCL high *and* SDA low

        //* SCL went low, SDA still low: end of start condition
        //* SDA went high, SCL still high : stop condition
        
        // Break when a stop condition has been received while waiting
        /** 
         * bug 1: This code does not reliably detect a stop condition, as it 
         * rereads SDA (which could have changed in between) and ignores SCL.
         * Together with bug 2 below, it would crash the device when used with
         * an ESP8266 and addresses >= 0x40 because of a very thight, but still
         * I2C conform, timing of the ESP8266 Arduino core's Wire implementation.
        */
        if (PINB & 32) {
          //* bug 2: this exits the main while(1) loop and the main() function
          break; 
        }
       
        // Clear i2c status flags and prepare to receive the device address
        USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
        USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);

        // Wait for byte received or stop condition
        while (!(USISR & ((1<<USIOIF) | (1<<USIPF))));

        // If byte received
        //* possible bug 3: USISR/USIOIF is being reread
        if ((USISR & (1<<USIOIF))) {

            // If the address is 0 or matches the configured address
            // All bits are shifted 1 to the left as required by the i2c protocol.
            uint8_t myAddress=(ADR_b6<<7) | (ADR_b5<<6) | (ADR_b4<<5) | (PINB & 30); //* 30 = 0b00011110
            if (USIDR==0 || (USIDR==myAddress)) {

                // Switch activity LED on for 80ms
                PORTB &= ~64;
                ledTimeout=20; // 20*4ms

                // Send ack bit
                USIDR=0;
                DDRB |= 32;
                USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | 14;
                while (!(USISR & (1<<USIOIF)));
                DDRB &= ~32;  

                // Receive up to 10 bytes
                for (uint8_t channel=0; channel<10; channel++) {

                    // Clear i2c status flags and prepare to receive next byte
                    USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
                    USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);

                    // Wait for byte received or stop condition
                    while (!(USISR & ((1<<USIOIF) | (1<<USIPF))));

                    // If byte received
                    //* possible bug 4: USISR/USIOIF is being reread
                    if ((USISR & (1<<USIOIF))) {

                        // copy to the servo value array
                        servo[channel]=USIDR;

                        //Send ack bit
                        USIDR=0;
                        DDRB |= 32;
                        USISR = (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | 14;
                        while (!(USISR & (1<<USIOIF)));
                        DDRB &= ~32;
                    }

                    // else if stop condition received, break the for loop
                    else {
                       break; // for loop
                    }  
                }  
            }
        }    
    }
}
