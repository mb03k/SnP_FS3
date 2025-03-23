
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

volatile bool PD2_cs;
volatile bool PD3_cs;
volatile bool PB2_cs;

volatile bool PD2_ps = true;
volatile bool PD3_ps = true;
volatile bool PB2_ps = true;


void initGPIO() {
    // minuten
    DDRC |= 0b00111111;
    // stunden
    DDRD |= 0b10110011;
    //DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD4) | (1<<PD5) | (1 << PD7);
    DDRB |= (1 << PB0);
}

void initPWM() {
    // fast pwm, nicht invertierend
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64

    OCR0A = 128;
    OCR0B = 128;
}

void initButtons() {
    cli(); // deaktiviert interrupts

    // INT0 und INT1 setup
    DDRD &= ~(1 << PD2) & ~(1 << PD3); // Pins als Eingänge
    PORTD |= (1 << PD2) | (1 << PD3); // Pull-Up
    EICRA |= (1 << ISC01) | (1 << ISC11); // falling edge
    EIMSK |= (1 << INT0) | (1 << INT1); // externe Interrupts aktivieren

    // PB sleep button bleibt
    DDRB &= ~(1 << PB2);
    PORTB |= (1 << PB2); // pull up
    PCICR |= (1<<PCIE0); // Pin Change Interrupt Controll Register
    PCMSK0 |=(1<<PCINT2); // interrupt register aktivieren

    sei(); // aktiviert interrupts
}

int main() {
    initGPIO();
    initButtons();
    initPWM();

    while (1) {
    }
    return 0;
}

// Pin Change Interrupt for PB2 only (PCINT2)
ISR(PCINT0_vect) {
    PORTC ^= (1<<PC5);
    _delay_ms(400);
}

// INT0 - Button on PD2 (Minutes / Mode Switch)
ISR(INT0_vect) {
    PORTC ^= (1<<PC5);
    _delay_ms(400);
}

// INT1 - Button on PD3 (Hours)
ISR(INT1_vect) {
    PORTC ^= (1<<PC5);
    _delay_ms(400);
}
