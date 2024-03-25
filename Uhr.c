#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define F_CPU 32768UL

// LED Configurations
#define HOUR_LEDS_DDR    DDRD
#define HOUR_LEDS_PORT   PORTD
#define MINUTE_LEDS_DDR  DDRC
#define MINUTE_LEDS_PORT PORTC

// Button Configurations
#define BUTTON_DDR_B DDRB
#define BUTTON_PORT_B PORTB
#define BUTTON_PIN_B PINB
#define BUTTON1 _BV(PB5) // Button 1 on PB5
#define BUTTON2 _BV(PB6) // Button 2 on PB6

#define BUTTON_DDR_D DDRD
#define BUTTON_PORT_D PORTD
#define BUTTON_PIN_D PIND
#define BUTTON3 _BV(PD7) // Button 3 on PD7

// Time Structure
typedef struct {
    uint8_t hours;
    uint8_t minutes;
} Time;

volatile Time currentTime = {12, 0};
volatile uint8_t clockState = 1; // 1: "awake" (active), 0: "asleep" (Sleep-Mode)
volatile uint8_t seconds = 0;

// Function Prototypes
void initializeClock();
void updateTime();
void displayTime();
void setupPWMForBrightness();
void setupTimer2Asynchronous();
void setupWakeupInterrupts();
void toggleSleepMode();
void startupSequence();
void allLedsOn();
void allLedsOff();
uint8_t debounceButtonB(uint8_t button);
uint8_t debounceButtonD(uint8_t button);

ISR(TIMER2_OVF_vect) {
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        updateTime();
    }
}

ISR(PCINT0_vect) {
    // Pin-Change-Interrupt Service Routine
}

int main() {
    initializeClock();
    setupTimer2Asynchronous();
    sei(); // Enable global interrupts
    displayTime();

    while (1) {
        if (!clockState) {
            set_sleep_mode(SLEEP_MODE_ADC);
            sleep_mode();
        }
        if (debounceButtonB(BUTTON1)) {
            _delay_ms(50);
            toggleSleepMode();
        }
        if (debounceButtonB(BUTTON2)) {
            currentTime.minutes = (currentTime.minutes + 1) % 60;
            if (clockState) {
                displayTime();
            }
        }
        if (debounceButtonD(BUTTON3)) {
            currentTime.hours = (currentTime.hours + 1) % 24;
            if (clockState) {
                displayTime();
            }
        }

        if (debounceButtonD(BUTTON3) && debounceButtonB(BUTTON2)) {
            // To-Do
        }
    }
}

void initializeClock(void) {
    HOUR_LEDS_DDR |= 0x1F; // Hour LEDs as output
    MINUTE_LEDS_DDR |= 0x3F; // Minute LEDs as output

    BUTTON_DDR_B &= ~((1 << PB5) | (1 << PB6)); // Buttons on PB5 and PB6 as input
    BUTTON_PORT_B |= (BUTTON1 | BUTTON2); // Enable pull-up resistors for buttons

    BUTTON_DDR_D &= ~(1 << PD7); // Button on PD7 as input
    BUTTON_PORT_D |= BUTTON3; // Enable pull-up resistor for button

    PCICR |= (1 << PCIE0); // Enable Pin-Change Interrupts
    PCMSK0 |= (1 << PCINT7); // Enable Pin-Change Interrupts for PD7
}

void updateTime() {
    currentTime.minutes++;
    if (currentTime.minutes >= 60) {
        currentTime.minutes = 0;
        currentTime.hours = (currentTime.hours + 1) % 24;
    }
    if (clockState) {
        displayTime();
    }
}

void displayTime() {
    // Turn off all LEDs
    HOUR_LEDS_PORT &= ~0x1F;
    MINUTE_LEDS_PORT &= ~0x3F;

    // Set hour LEDs
    for (uint8_t i = 0; i < 5; i++) {
        if (currentTime.hours & (1 << i)) {
            HOUR_LEDS_PORT |= (1 << i);
        }
    }

    // Set minute LEDs
    for (uint8_t i = 0; i < 6; i++) {
        if (currentTime.minutes & (1 << i)) {
            MINUTE_LEDS_PORT |= (1 << i);
        }
    }
}

void setupPWMForBrightness() {
    // To-Do
}

void setupTimer2Asynchronous() {
    ASSR |= (1 << AS2); // Enable asynchronous mode for Timer2
    TCCR2A = 0; // Normal mode
    TCCR2B = (1 << CS22) | (1 << CS20); // Set prescaler to 128
    while (ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << OCR2BUB) | (1 << TCR2AUB) | (1 << TCR2BUB)));
    TIMSK2 = (1 << TOIE2); // Enable Timer/Counter2 Overflow Interrupt
}

uint8_t debounceButtonB(uint8_t button) {
    _delay_ms(50);
    return !(BUTTON_PIN_B & button);
}

uint8_t debounceButtonD(uint8_t button) {
    _delay_ms(50);
    return !(BUTTON_PIN_D & button);
}

void toggleSleepMode(void) {
    if (clockState) {
        // Turn off LEDs
        HOUR_LEDS_PORT &= ~(0x1F);
        MINUTE_LEDS_PORT &= ~(0x3F);
        clockState = 0; // Set clock state to "asleep"
    } else {
        // Turn on LEDs according to current time
        displayTime();
        clockState = 1; // Set clock state to "awake"
    }
}


