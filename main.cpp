
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

void handleTimeChange();
void handleHours();

void initPWM() {
    // fast pwm, nicht invertierend
    /*TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
    TCCR0A |= (1 << COM0A1); // Nicht-invertierend auf OC0A
    TCCR0A |= (1 << COM0B1); // Nicht-invertierend auf OC0A
    TCCR0B |= (1 << CS01) | (1 << CS00);  // Prescaler 64*/

    TCCR0A = (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1); // 8-bit Fast PWM, Clear on Compare Match
    TCCR0B = (1 << CS01); // ps = 8

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
    /*set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    // CPU wakes up here after an interrupt
    sleep_disable(); // Disable sleep mode after wake-up*/

    if (deepsleep) { // PWR_DOWN
        // set sleep mode
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
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

int main() {
    cli(); // interrupts deaktivieren
    initPWM();
    initQuartz();
    initGPIO();
    initButtons();
    sei(); // interrupts aktivieren

    while (1) {
        // hier testen wie oft die schleife iteriert wird?
        // sollte nicht so oft sein. Eigentlich nur bei einem interrupt
        // PORTD ^= (1<<PD4); // Jede Sekunde aufgerufen
        //sleep_mode();
        initSleep();
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

void refreshLEDs_min() {
    for (int i=0; i<6; i++) {
        int minVal = (minutes >> i) & 0b1; // Wert zum prüfen ob die LED an oder aus sein soll

        if (minVal) {
            PORTC |= (1 << i);
        } else {
            PORTC &= ~(1 << i);
        }
    }
}

void refreshLEDs_hours() {
    for (int i=0; i<5; i++) {
        int hoursVal = (hours >> i) & 0b1; // Wert zum Prüfen, ob die LED an oder aus sein soll
        int ledIndex = 4 - i; // LEDs in umgekehrter Reihenfolge anschalten

        if (hoursVal) {
            PORTD |= (1 << ledIndex);
        } else {
            PORTD &= ~(1 << ledIndex);
        }
    }
}

/* ---------------------- */


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

void btnPressedStandardMode() {
    /*
     * Prüfen ob ein Zustandswechsel gemacht werden soll
     * (PB0 wird mehr als 2 s gehalten)
     */
    if (!risingEdge(0)) { // Button wurde runter gedrückt
        btnPressedTimer = 0;
    }
    else if (risingEdge(0)){ // Button wurde losgelassen
        if (btnPressedTimer >= 2) { // Modi wechseln
            IN_STANDARD_MODI = !IN_STANDARD_MODI;
        } else {
            seconds = 0;
            minutes++;
            handleTimeChange();
        }
    }

    if (risingEdge(1)) { // hour
        seconds = 0;
        hours++;
        handleHours();
    } else if (risingEdge(2)) {
        PORTC^=(1<<PC4);
        deepsleep = !deepsleep;
    }
}

void btnPressedExperimentMode() {
    /*
     * Prüfen ob Zustandswechsel erfolgen soll
     */
    if (!risingEdge(0)) { // Button wurde runter gedrückt
        btnPressedTimer = 0;
    }
    else if (risingEdge(0)){ // Button wurde losgelassen
        if (btnPressedTimer >= 2) { // Modi wechseln
            IN_STANDARD_MODI = !IN_STANDARD_MODI;
        } else {
            seconds = 0;
            minutes--;
            handleTimeChange();
        }
    }

    if (risingEdge(1)) { // hour
        seconds = 0;
        hours--;
        handleHours();
    } else if (risingEdge(2)) { // PWM heller stellen
        pwm_brightness_level_counter++;
        if (pwm_brightness_level_counter>=7) {
            pwm_brightness_level_counter = 0;
        }
        OCR0A = pwm_brightness_level[pwm_brightness_level_counter];
        OCR0B = pwm_brightness_level[pwm_brightness_level_counter];
    }
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