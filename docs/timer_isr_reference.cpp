// timer_isr_reference.cpp
// 
// Hardware timer interrupt reference implementation for nRF52840.
// Not used in current build because Mbed OS reserves TIMER2/3/4.
// In a bare-metal (no RTOS) environment, this would replace the
// millis()-based scheduler for deterministic jitter-free sampling.
//
// To use: remove Mbed OS framework, build with nRF5 SDK or Zephyr.

#include <nrf52840.h>

volatile bool sample_ready = false;
volatile uint32_t isr_count = 0;

// ISR keeps it minimal — just sets a flag, no I2C in interrupt context
extern "C" void TIMER3_IRQHandler(void) {
    if (NRF_TIMER3->EVENTS_COMPARE[0]) {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;
        sample_ready = true;
        isr_count++;
    }
}

void timer_init(uint16_t freq_hz) {
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;

    NRF_TIMER3->MODE = 0;        // Timer mode (not counter)
    NRF_TIMER3->BITMODE = 3;     // 32-bit width
    NRF_TIMER3->PRESCALER = 9;   // 16MHz / 2^9 = 31250 Hz base clock

    // Compare value: 31250 / freq_hz
    // For 50Hz: 31250 / 50 = 625 ticks = exactly 20ms
    NRF_TIMER3->CC[0] = 31250 / freq_hz;

    // Shortcut: auto-clear counter on compare match (periodic mode)
    NRF_TIMER3->SHORTS = (1 << 0);  // COMPARE0_CLEAR

    // Enable COMPARE0 interrupt
    NRF_TIMER3->INTENSET = (1 << 16);

    // Set priority and enable in NVIC
    NVIC_SetPriority(TIMER3_IRQn, 2);  // High priority
    NVIC_EnableIRQ(TIMER3_IRQn);

    NRF_TIMER3->TASKS_START = 1;
}

// Usage in main loop:
//
// void loop() {
//     if (sample_ready) {
//         sample_ready = false;
//         // read IMU, filter, process...
//     }
// }