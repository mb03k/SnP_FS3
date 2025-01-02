
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

volatile bool previousRisingEdge_pb0 = false;
volatile bool previousRisingEdge_pb1 = false;
volatile bool previousRisingEdge_pb2 = false;

uint8_t MIN_BR = 250;
uint8_t HOUR_BR = 250;

void handleTimeChange();
void handleMinutes();
void handleHours();
void refreshLEDs_min();
void refreshLEDs_hours();

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
void initTimer() {
    ASSR |= (1 << AS2); // Uhrenquarz asynchron zum CPU takt laufen lassen
    TCCR2B |= (1 << CS22) | (1 << CS20); // ps 128
    TIMSK2 |= (1 << TOIE2);
    while (ASSR & ((1 << TCR2BUB) | (1 << OCR2BUB) | (1 << TCN2UB)));
}

void initGPIO() {
    DDRC |= 0b00111111; // minuten
    DDRD |= 0b11111111; // stunden
}

void initButtons() {
    PCICR |= (1<<PCIE0); // controll register für PD7:0 aktivieren
    PCIFR |= (1<<PCIF0);

    // PCINT1 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB1);
    PORTB |= (1<<PB1);
    // interrupt
    PCMSK0 |= (1<<PCINT1); // mask aktivieren

    // PCINT2 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB2);
    PORTB |= (1<<PB2);
    // interrupt
    PCMSK0 |= (1<<PCINT2);

    // PCINT0 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB0);
    PORTB |= (1<<PB0);
    // interrupt
    PCMSK0 |= (1<<PCINT0);
}

void initSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
}

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initTimer();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    PORTD |= (1<<PD7);

    initSleep();

    while (1) {
        sleep_mode();
    }
    return 0;
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
    //PORTD ^=(1<<PD7);
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        //handleTimeChange();
    }
}

ISR(PCINT0_vect) {
    uint8_t pb0_interrupt = (PINB & 0b1);
    uint8_t pb1_interrupt = (PINB >> 1) & 0b1;
    uint8_t pb2_interrupt = (PINB >> 2) & 0b1;
PORTD ^=(1<<PD7);
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
    /*else if (!pb2_interrupt && !previousRisingEdge_pb2) {
        PORTD ^= (1<<PD7);
        seconds = 0;
        minutes = 0;
        hours = 0;
        refreshLEDs_min();
        refreshLEDs_hours();

        previousRisingEdge_pb2 = pb2_interrupt;
    }*/
}