#define F_CPU 1000000UL  // Taktfrequenz des Mikrocontrollers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/delay.h>


void init_timer() {
    ASSR |= (1 << AS2); // Externer Quarz für Timer2
    TCCR2B |= (1 << CS22) | (1 << CS20); // Prescaler 128
    TIMSK2 |= (1 << TOIE2); // Overflow-Interrupt aktivieren
    while (ASSR & ((1 << TCR2BUB) | (1 << OCR2BUB) | (1 << TCN2UB))); // Warten auf Synchronisierung
}

void initPWM() {
    DDRD |= (1 << PD6);  // OC0A als Ausgang
    DDRD |= (1 << PD5);  // OC0A als Ausgang
    // Fast PWM, nicht-invertierend
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64
    OCR0A = 240;  // 50% Duty Cycle
    OCR0B = 240;
}

int main() {
    init_timer();
    initPWM();
    cli(); // Interrupts deaktivieren

    DDRD |= (1<<PD1);
    DDRD |= (1<<PD7);
    PORTD |= (1<<PD1);
    sei(); // Interrupts aktivieren

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);

    while (1) {
        sleep_mode();
    }
}

ISR(TIMER2_OVF_vect) {
    PORTD ^= (1 << PD1); // Status-LED toggeln
}
