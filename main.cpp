
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

bool IN_STANDARD_MODI = true;

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

volatile bool PB0_ps = false;
volatile bool PB1_ps = false;
volatile bool PB2_ps = false;

volatile bool PB0_cs = false;
volatile bool PB1_cs = false;
volatile bool PB2_cs = false;

volatile uint8_t btnPressedTimer = 0;

uint8_t MIN_BR = 254;
uint8_t HOUR_BR = 254;

void handleTimeChange();
void handleHours();

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
    DDRC |= 0b00111111; // minuten
    DDRD |= 0b11111111; // stunden
}

void initButtons() {
    PCICR |= (1<<PCIE0); // controll register für PD7:0 aktivieren
    PCIFR |= (1<<PCIF0);

    // PCINT0 bzw. Pull-Up aktivieren
    DDRB &= ~(1<<PB0);
    PORTB |= (1<<PB0);
    // interrupt
    PCMSK0 |= (1<<PCINT0);

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
}

void initSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    // CPU wakes up here after an interrupt
    sleep_disable(); // Disable sleep mode after wake-up
}

void setPWRDown() {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    sleep_cpu();
}

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initQuartz();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    initSleep();

    while (1) {
        // hier testen wie oft die schleife iteriert wird?
        // sollte nicht so oft sein. Eigentlich nur bei einem interrupt
        // PORTD ^= (1<<PD4); // Jede Sekunde aufgerufen
        sleep_mode();
    }
    return 0;
}



/*
 *  Button gedrückt / eine Minute vergangen:
 *  LEDs aktualisieren
 */
void handleMinutes();
void refreshLEDs_min();
void refreshLEDs_hours();

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

/* ---------------------- */



void btnPressedStandardMode() {
    if (!PB0_ps && PB0_cs) {
        PORTC ^=(1<<PC1);
    }
}

void btnPressedExperimentMode() {

}

// eine Sekunde vergangen
ISR(TIMER2_OVF_vect) {
    PORTD ^= (1<<PD3);
    btnPressedTimer++;
    if (btnPressedTimer >= 10) {
        btnPressedTimer = 0;
    }
}

// Button wurde gedrückt
ISR(PCINT0_vect) {
    PB0_cs = (PINB & 0b1);
    PB1_cs = (PINB >> 1) & 0b1;
    PB2_cs = (PINB >> 2) & 0b1;


    /*
     * Prüfen ob ein Zustandswechsel gemacht werden soll
     * (PB0 wird mehr als 2 s gehalten
     */
    if ( PB0_ps && !PB0_cs) { // Button wurde runter gedrückt
        btnPressedTimer = 0;
    }
    else if ( !PB0_ps && PB0_cs ){ // Button wurde losgelassen
        if (btnPressedTimer >= 2) { // Modi wechseln
            PORTC ^= (1 << PC4);
            // changeMode();
        } else {
            if (IN_STANDARD_MODI) {
                btnPressedStandardMode();
            } else {
                btnPressedExperimentMode();
            }
        }
    }

    PB0_ps = PB0_cs;
    PB1_ps = PB1_cs;
    PB2_ps = PB2_cs;
}