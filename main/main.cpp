
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

volatile bool PB0_cs;
volatile bool PB1_cs;
volatile bool PB2_cs;

volatile bool deepsleep = false;

// mit true initialisieren da sonst beim ersten Press die if-Bedingungen immer als True angenommen werden
volatile bool PB0_ps = true;
volatile bool PB1_ps = true;
volatile bool PB2_ps = true;

uint8_t MIN_BR = 254;
uint8_t HOUR_BR = 254;

void initPWM() {
    // fast pwm, nicht invertierend
    /*TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64*/

    TCCR0A = (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1); // 8-bit Fast PWM, Clear on Compare Match
    TCCR0B = (1 << CS01); // ps = 8
    //TCCR0B = (1 << CS00) | (1 << CS01); // Prescaler = 8

    OCR0A = MIN_BR;
    OCR0B = HOUR_BR;
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

void enter_sleep();

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    while (1) {
        if (deepsleep) { // PWR_DOWN
            // set sleep mode
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_enable();
            sei();
            sleep_cpu();
            PORTC^=(1<<PC2);
        } else {
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sei();
            sleep_cpu();
            PORTC^=(1<<PC0);
        }
        sleep_disable();
    }
    return 0;
}



bool risingEdge(uint8_t PB) {
    if (!PB) {
        return !PB0_ps && PB0_cs;
    } else if (PB == 1) {
        return !PB1_ps && PB1_cs;
    } else if (PB == 2) {
        return !PB2_ps && PB2_cs;
    }
    return false;
}


// Button wurde gedrückt
ISR(PCINT0_vect) {
    PB0_cs = (PINB & 0b1);
    PB1_cs = (PINB >> 1) & 0b1;
    PB2_cs = (PINB >> 2) & 0b1;

    // irgendein Button wird losgelassen
    if (risingEdge(0)) {
        PORTD^=(1<<PD0);
    }
    else if (risingEdge(1)) {
        PORTD^=(1<<PD1);
    }
    else if (risingEdge(2)) {
        deepsleep = !deepsleep;
    }

    PB0_ps = PB0_cs;
    PB1_ps = PB1_cs;
    PB2_ps = PB2_cs;
}