
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

bool IN_STANDARD_MODI = true;

volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;

volatile bool PB0_cs;
volatile bool PB1_cs;
volatile bool PB2_cs;

// mit true initialisieren da sonst beim ersten Press die if-Bedingungen immer als True angenommen werden
volatile bool PB0_ps = true;
volatile bool PB1_ps = true;
volatile bool PB2_ps = true;

bool deepsleep = false;


volatile uint8_t pwm_brightness_level[] = {254, 250, 240, 210, 180, 140, 80, 0};
volatile uint8_t pwm_brightness_level_counter = 0;

volatile uint8_t btnPressedTimer = 0;

uint8_t MIN_BR = 254;
uint8_t HOUR_BR = 254;

void initPWM() {
    /*
     * Schauen welche Funktion besser funktioniert
     */
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

    // Register bzw. Pull-Ups aktivieren
    DDRB &= ~(1<<PB0) | ~(1<<PB1) | ~(1<<PC2); // register
    PORTB |= (1<<PB0) | (1<<PB1) | (1<<PB2); // pull-up

    // interrupt register aktivieren
    PCMSK0 |= (1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2);
}

void initSleep() {
    if (deepsleep) {
        // set sleep mode
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sei();
        sleep_cpu();
    } else {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();
    }
    sleep_disable(); // wacht hier bei interrupt auf
}

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initQuartz();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    while (1) {
        initSleep();
    }
    return 0;
}

// eine Sekunde vergangen
ISR(TIMER2_OVF_vect) {
    btnPressedTimer++;
    if (btnPressedTimer >= 10) {
        btnPressedTimer = 0;
    }

    // LED blinken lassen um Zeit zu messen
    if (!IN_STANDARD_MODI) {
        PORTD ^= (1<<PD4);
    }
}

// Button wurde gedrückt
ISR(PCINT0_vect) {
    PB0_cs = (PINB & 0b1);
    PB1_cs = (PINB >> 1) & 0b1;
    PB2_cs = (PINB >> 2) & 0b1;

    // irgendein Button wurde gedrückt
    if (IN_STANDARD_MODI) {
        btnPressedStandardMode();
    } else {
        btnPressedExperimentMode();
    }

    PB0_ps = PB0_cs;
    PB1_ps = PB1_cs;
    PB2_ps = PB2_cs;
}