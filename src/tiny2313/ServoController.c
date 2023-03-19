#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

// Servo controller with I2C interface.
// Generates PWM signals for up to 10 servos.

// Written for ATtiny2313, using internal 4Mhz RC oscillator

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

// The input pins for i2c address bits have internal pull-up's.
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
    PORTD &= ~1;
    PORTD &= ~2;
    PORTA &= ~2;
    PORTA &= ~1;
    PORTD &= ~4;
}

// Timer 0 Compare B Interrupt
ISR(TIMER0_COMPB_vect) {    
    // Sets servo 5-9 pins to low
    PORTD &= ~8;
    PORTD &= ~16;
    PORTD &= ~32;
    PORTD &= ~64;
    PORTB &= ~1;
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
    OCR0A=servo[loop];
    OCR0B=servo[loop+5];

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
    DDRA  = 1|2;
    DDRB  = 1|64;
    PORTB = 2|4|8|16;
    DDRD  = 1|2|4|8|16|32|64;

    
    // Timer 0: fast PWM, prescaler 64, IRQ on overflow, compare match A and compare match B.
    TCCR0A = (1<<WGM01) | (1<<WGM00);
    TCCR0B = (1<<CS01)  | (1<<CS00);
    TIMSK  = (1<<TOIE0) | (1<<OCIE0A) | (1<<OCIE0B);

    // Enable interrupts
    sei();

    // Receive data from I2C in the main loop
    while (1) {

        // Enable driving SCL
        DDRB |= 128;
        PORTB |= 128;    
        // SDA=input
        DDRB &= ~32;    

        // Clear i2c status flags and wait for start condition
        USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
        USICR = (1<<USIWM1) | (1<<USICS1);
        while (!(USISR & (1<<USISIF)));
   
        // Wait until end of start condition or begin of stop condition 
        while ((PINB & 128) && !(PINB & 32));

        // Break when a stop condition has been received while waiting
        if (PINB & 32) {
          break;
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
            uint8_t myAddress=(ADR_b6<<7) | (ADR_b5<<6) | (ADR_b4<<5) | (PINB & 30);
            if (USIDR==0 || (USIDR==myAddress)) {

                // Switch activity LED for 80ms on
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
                       break;
                    }  
                }  
            }
        }    
    }
}
