#define F_CPU 1000000UL  // Taktfrequenz des Mikrocontrollers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/delay.h>

uint8_t MIN_BR = 250;
uint8_t HOUR_BR = 250;

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

volatile uint8_t previousRisingEdge_pb0 = 0;
volatile uint8_t previousRisingEdge_pb1 = 0;
volatile uint8_t previousRisingEdge_pb2 = 0;

void handleTimeChange();
void handleMinutes();
void handleHours();

void refreshLEDs_min();
void refreshLEDs_hours();

void goToSleep();

void init_buttons() {
    // PCINT1 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB1);
    PORTB |= (1<<PB1);
    // interrupt
    PCICR |= (1<<PCIE0); // control register
    PCMSK0 |= (1<<PCINT1); // mask aktivieren

    // PCINT2 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB2);
    PORTB |= (1<<PB2);
    // interrupt
    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT2);

    // PCINT0 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB0);
    PORTB |= (1<<PB0);
    // interrupt
    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT0);
}

// Zeitbasis 1s
void init_timer() {
    ASSR |= (1 << AS2);
    TCCR2B |= (1 << CS22) | (1 << CS20); // ps 128
    TIMSK2 |= (1 << TOIE2);
    while (ASSR & ((1 << TCR2BUB) | (1 << OCR2BUB) | (1 << TCN2UB)));
}

// LED Helligkeit
void initPWM() {
    DDRD |= (1 << PD6);  // OC0A als Ausgang
    // Fast PWM, nicht-invertierend
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64
    OCR0A = MIN_BR;  // 50% Duty Cycle
    OCR0B = HOUR_BR;
}

void goToSleep() {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

int main() {
    init_buttons();
    init_timer();
    initPWM();
    cli(); // interrupts deaktivieren

    DDRC |= 0b00111111; // minuten
    DDRD |= 0b00011111; // stunden
    DDRD |= (1<<PD7);

    sei(); // interrupts aktivieren

    //goToSleep();

    while (1) {
        //sleep_mode();
        //_delay_ms(500);
    }
}

void handleTimeChange() {
    handleMinutes();
    handleHours();
}

void handleMinutes() {
    if (minutes >= 60) {
        minutes = 0;
        hours++;
    }
    refreshLEDs_min();
}

void handleHours() {
    if (hours >= 24) {
        hours = 0;
        PORTD |=(1<<PD7);
    }
    refreshLEDs_hours();
}

void refreshLEDs_min() {
    for (int i=0; i<8; i++) {
        int minVal = (minutes >> i) & 0b1;

        if (minVal) {
            PORTC |= (1 << i);
        } else {
            PORTC &= ~(1 << i);
        }
    }
}

void refreshLEDs_hours() {
    for (int i=0; i<5; i++) {
        int hoursVal = (hours >> i) & 0b1;

        if (hoursVal) {
            PORTD |= (1 << i);
        } else {
            PORTD &= ~(1 << i);
        }
    }
}

ISR(TIMER2_OVF_vect) {
    seconds++;
    PORTD ^= (1<<PD7);
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        handleTimeChange();
    }
}

ISR(PCINT0_vect) {
    uint8_t pb0_interrupt = (PINB & 0b1);
    uint8_t pb1_interrupt = (PINB >> 1) & 0b1;
    uint8_t pb2_interrupt = (PINB >> 2) & 0b1;

    // MINUTES
    if (!pb0_interrupt && !previousRisingEdge_pb0) {
        seconds = 0;
        minutes++;
        handleTimeChange();

        previousRisingEdge_pb0 = pb0_interrupt;
    }
    // HOURS
    else if (!pb1_interrupt && !previousRisingEdge_pb1) {
        seconds = 0;
        hours++;
        handleHours();

        previousRisingEdge_pb1 = pb1_interrupt;
    }
    // RESET
    else if (!pb2_interrupt && !previousRisingEdge_pb2) {
        PORTD ^= (1<<PD7);
        seconds = 0;
        minutes = 0;
        hours = 0;
        refreshLEDs_min();
        refreshLEDs_hours();

        previousRisingEdge_pb2 = pb2_interrupt;
    }
}