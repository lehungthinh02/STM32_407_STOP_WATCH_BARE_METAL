#include "stm32f407xx.h"           // Includes definitions for STM32F407xx
#include "stm32f4xx_gpio.h"        // Includes GPIO driver functions
#include "stm32f4xx_gp_timer.h"    // Includes Timer driver functions

#define TIM4_IRQn           30     // Interrupt number for TIM4 in NVIC
#define BUTTON_RESET_PIN    13     // PC13 for reset button
#define BUTTON_START_PIN    14     // PC14 for start/stop button

// Global variables to track stopwatch state and counters
volatile uint8_t stopwatch_running = 0;     // Variable to check if stopwatch is running or stopped
volatile uint16_t integer_counter = 0;      // Seconds counter (0-59)
volatile uint16_t decimal_counter = 0;      // Milliseconds counter (0-999)
volatile uint8_t digits[5] = {0};           // Array of digits for 5-digit display (2 for seconds, 3 for milliseconds)
volatile uint8_t current_digit_index = 0;   // Index of the current digit being displayed

// Debounce variables
volatile uint8_t last_reset_button_state = 1;
volatile uint8_t last_start_button_state = 1;

// Segment codes for digits 0-9 for common cathode 7-segment display (active low)
uint8_t segmentMap[] = {
    0xC0, // 0
    0xF9, // 1
    0xA4, // 2
    0xB0, // 3
    0x99, // 4
    0x92, // 5
    0x82, // 6
    0xF8, // 7
    0x80, // 8
    0x90  // 9
};

// Function prototypes for GPIO initialization for 7-segment display and Timer
void GPIO_Init_7Segment(void);
void GPIO_Init_Buttons(void);
void TIM4_Init(void);
void TIM4_IRQHandler(void);
void NVIC_EnableIRQ(uint8_t IRQNumber);

int main(void)
{
    // Initialize GPIO for 7-segment display and buttons
    GPIO_Init_7Segment();
    GPIO_Init_Buttons();
    TIM4_Init();  // Initialize Timer to generate periodic interrupts

    __asm volatile ("cpsie i");  // Enable interrupts globally

    while (1)
    {
        // Read the current state of the reset button (with debouncing)
        uint8_t current_reset_button_state = GPIO_ReadFromInputPin(GPIOC, BUTTON_RESET_PIN);

        if (current_reset_button_state != last_reset_button_state)
        {
            // Debounce delay
            for (volatile uint32_t i = 0; i < 10000; i++);

            // Read the state again
            current_reset_button_state = GPIO_ReadFromInputPin(GPIOC, BUTTON_RESET_PIN);

            if (current_reset_button_state == GPIO_PIN_RESET)
            {
                // If reset button is pressed, reset the stopwatch
                integer_counter = 0;    // Reset seconds counter
                decimal_counter = 0;    // Reset milliseconds counter
            }

            last_reset_button_state = current_reset_button_state;
        }

        // Read the current state of the start/stop button (with debouncing)
        uint8_t current_start_button_state = GPIO_ReadFromInputPin(GPIOC, BUTTON_START_PIN);

        if (current_start_button_state != last_start_button_state)
        {
            // Debounce delay
            for (volatile uint32_t i = 0; i < 10000; i++);

            // Read the state again
            current_start_button_state = GPIO_ReadFromInputPin(GPIOC, BUTTON_START_PIN);

            if (current_start_button_state == GPIO_PIN_RESET)
            {
                // If start/stop button is pressed, toggle the state of the stopwatch
                stopwatch_running = !stopwatch_running;  // Toggle the state
            }

            last_start_button_state = current_start_button_state;
        }

        // The actual counting is handled in the timer interrupt handler
    }
}

// GPIO initialization for buttons (PC13 and PC14)
void GPIO_Init_Buttons(void) {
    GPIO_Handle_t gpioButton;

    // Configure PC13 (BUTTON_RESET) with pull-up resistor
    gpioButton.pGPIOx = GPIOC;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = BUTTON_RESET_PIN;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Pull-up enabled
    GPIO_Init(&gpioButton);

    // Configure PC14 (BUTTON_START) with pull-up resistor
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = BUTTON_START_PIN;
    GPIO_Init(&gpioButton);
}

// GPIO initialization for 7-segment display (PA0-PA7) and digit control (PB0-PB4)
void GPIO_Init_7Segment(void) {
    // Enable clock for GPIOA and GPIOB
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Handle_t gpioSegment;
    gpioSegment.pGPIOx = GPIOA;
    gpioSegment.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;        // Set as output mode
    gpioSegment.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;    // Push-pull output type
    gpioSegment.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // No pull-up/pull-down
    gpioSegment.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;     // High speed

    // Configure segment control pins (PA0-PA7)
    for (uint8_t pin = 0; pin <= 7; pin++) {
        gpioSegment.GPIO_PinConfig.GPIO_PinNumber = pin;
        GPIO_Init(&gpioSegment);
    }

    GPIO_Handle_t gpioDigit;
    gpioDigit.pGPIOx = GPIOB;
    gpioDigit.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;          // Set as output mode
    gpioDigit.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;      // Push-pull output type
    gpioDigit.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;    // No pull-up/pull-down
    gpioDigit.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;       // High speed

    // Configure digit select pins (PB0-PB4) for 5 digits
    for (uint8_t pin = 0; pin <= 4; pin++) {
        gpioDigit.GPIO_PinConfig.GPIO_PinNumber = pin;
        GPIO_Init(&gpioDigit);
    }
}

// Timer initialization for periodic interrupt
void TIM4_Init(void) {
    // Enable clock for TIM4
    RCC->APB1ENR |= (1 << 2);  // Bit 2 corresponds to TIM4

    // Set prescaler and auto-reload values for 1ms interval (1 kHz)
    uint32_t pclk1 = 16000000;  // Assuming 16 MHz APB1 clock

    // Set prescaler to get 1 MHz timer clock (i.e., 1 us period)
    TIM4->PSC = (pclk1 / 1000000) - 1;  // Prescaler value for 1 MHz

    // Set auto-reload value for 1 ms interrupt
    TIM4->ARR = 1000 - 1;  // 1ms interval

    // Enable update interrupt for TIM4
    TIM4->DIER |= (1 << 0);  // UIE: Update Interrupt Enable
    // Start Timer
    TIM4->CR1 |= (1 << 0);  // CEN: Counter Enable
    // Enable TIM4 interrupt in NVIC
    NVIC_EnableIRQ(TIM4_IRQn);
}

// Timer interrupt handler (for 1ms timer updates)
void TIM4_IRQHandler(void) {
    if (TIM4->SR & 0x0001) {  // Check if update interrupt flag is set
        TIM4->SR &= ~(1 << 0);  // Clear update interrupt flag (UIF)

        // Only increment counters if stopwatch is running
        if (stopwatch_running) {
            // Update decimal counter (1 ms increments)
            decimal_counter++;  // Increment decimal counter by 1 (each interrupt is 1 ms)

            // If the decimal part reaches 1000 (i.e., 1 second), reset and increment seconds
            if (decimal_counter >= 1000) {
                decimal_counter = 0;    // Reset decimal counter
                integer_counter++;      // Increment the integer counter (seconds)
                if (integer_counter >= 60) {
                    integer_counter = 0; // Reset seconds after 60
                }
            }
        }

        // Split the counter value into digits for display
        uint16_t tempInteger = integer_counter;
        uint16_t tempDecimal = decimal_counter;

        // Extract digits for the integer part (seconds)
        digits[0] = tempInteger / 10;           // Tens place
        digits[1] = tempInteger % 10;           // Ones place

        // Extract digits for the decimal part (milliseconds)
        digits[2] = (tempDecimal / 100) % 10;   // Hundreds place
        digits[3] = (tempDecimal / 10) % 10;    // Tens place
        digits[4] = tempDecimal % 10;           // Ones place

        // Multiplexing the display for the 7-segment display
        GPIO_WriteToOutputPort(GPIOB, 0x00);  // Turn off all digits

        // Send the segment data to the 7-segment display (PA0-PA7)
        uint8_t segmentData = segmentMap[digits[current_digit_index]];
        GPIO_WriteToOutputPort(GPIOA, segmentData);

        // Activate the current digit (active low)
        GPIO_WriteToOutputPin(GPIOB, current_digit_index, GPIO_PIN_SET);

        // Move to the next digit
        current_digit_index++;
        if (current_digit_index >= 5) {
            current_digit_index = 0;  // Reset the digit index after the 5th digit
        }
    }
}

// Function to enable NVIC interrupt for a given IRQ number
void NVIC_EnableIRQ(uint8_t IRQNumber) {
    if(IRQNumber <= 31) {
        *NVIC_ISER0 |= (1 << IRQNumber);           // Enable interrupt in ISER0
    } else if(IRQNumber <= 63) {
        *NVIC_ISER1 |= (1 << (IRQNumber % 32));    // Enable interrupt in ISER1
    } else if(IRQNumber <= 95) {
        *NVIC_ISER2 |= (1 << (IRQNumber % 64));    // Enable interrupt in ISER2
    }
}
