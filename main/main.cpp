
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>


uint8_t MIN_BR = 254;
uint8_t HOUR_BR = 254;

volatile bool PB2_cs = true;
volatile bool PB2_ps = true;

volatile uint16_t schaltsekunden = 0;
volatile uint8_t SLEEP_AFTER_INACTIVITY = 0;

void initPWM() {
    // fast pwm, nicht invertierend
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64

    OCR0A = MIN_BR;
    OCR0B = HOUR_BR;
}

// Zeitbasis 1s
void initQuartz() {
    ASSR |= (1 << AS2); // Uhrenquarz asynchron zum CPU takt laufen lassen
    TCCR2B |= (1 << CS22) | (1 << CS20); // ps 128
    TIMSK2 |= (1 << TOIE2);
}

void initGPIO() {
    // minuten
    DDRC |= (1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5);

    // stunden
    DDRD |= (1<<PD0) | (1<<PD1) | (1<<PD4) | (1<<PD7);
    DDRB |= (1<<PB0);

    // PWM -> egal ob in- oder output
    DDRD |= (1<<PD5) | (1<<PD6);

    // taster
    DDRD &= ~(1<<PD2);
    DDRD &= ~(1<<PD3);
}

void initButtons() {
    // INT0 und INT1 setup
    DDRD &= ~(1 << PD2) & ~(1 << PD3); // Pins als Eingänge
    PORTD |= (1 << PD2) | (1 << PD3); // Pull-Up
    EICRA |= (1 << ISC10) | (1 << ISC11); // falling edge
    EIMSK |= (1 << INT0) | (1 << INT1); // externe Interrupts aktivieren

    // PB sleep button bleibt
    DDRB &= ~(1 << PB2);
    PORTB |= (1 << PB2); // pull up
    PCICR |= (1<<PCIE0); // Pin Change Interrupt Controll Register
    PCMSK0 |=(1<<PCINT2); // interrupt register aktivieren
}

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    while (1) {
    }
    return 0;
}

// Button wurde gedrückt
ISR(PCINT0_vect) {
    SLEEP_AFTER_INACTIVITY = 0;
    PB2_cs = (PINB >> 2) & 0b1;

    if (!PB2_ps && PB2_cs) { // rising edge
        PORTC ^= (1<<PC5);
    }

    PB2_ps = PB2_cs;
}

// INT0
ISR(INT0_vect) {
    PORTC ^= (1<<PC4);
}

// INT1
ISR(INT1_vect) {
    PORTC ^= (1<<PC3);
}