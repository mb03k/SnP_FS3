#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

uint8_t MIN_BR = 10;
uint8_t HOUR_BR = 10;

void lightUpLEDs();

void initPWM() {
    // Fast PWM, nicht invertierend
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0B
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64
    OCR0A = MIN_BR;
    OCR0B = MIN_BR;
}

void initGPIO() {
    DDRC |= 0b00111111; // Set PC0 to PC5 as output (minutes)
    DDRD |= 0b11111111; // Set PD0 to PD7 as output (hours)
}

int main() {
    initGPIO();
    initPWM();

    // Turn on LEDs
    lightUpLEDs();

    while (1) {}
    return 0;
}

void lightUpLEDs() {
    PORTD |= 0b00011111; // Set PD0 to PD4
    PORTC |= 0b00111111; // Set PC0 to PC5
}
