#include "inc/tm4c123gh6pm.h"
#include <stdio.h>

// Pin Definitions
#define FLAME_SENSOR_PIN PB_5  // Use analog pin for flame sensor
#define BUZZER_PIN PA_2 // Output pin for buzzer
#define ESP32_TRIGGER_PIN PE_0 // Esp32 Trigger pin

// Threshold for flame detection
const int flameThreshold = 2048; // Adjusted threshold, based on a 12-bit ADC

// Time constants
const unsigned long detectionThresholdTime = 5000; // 5 seconds in milliseconds

// Variables to track flame detection
unsigned long fireStartTime = 0;
bool fireDetected = false;
bool espTriggered = false;

// UART0 Configuration
void UART0_Init() {
  // Enable UART0 and GPIOA peripheral
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // Enable UART0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // Enable GPIO Port A

  // Disable UART0 before configuration
  UART0_CTL_R &= ~UART_CTL_UARTEN;

  // Configure GPIO Pins for UART functionality
  GPIO_PORTA_AFSEL_R |= 0x03; // Enable alternate function on PA0 and PA1
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x11; // Configure PA0 and PA1 for UART
  GPIO_PORTA_DEN_R |= 0x03; // Enable digital I/O on PA0 and PA1

  // Set baud rate to 9600
  // Baud rate = System Clock / (16 * baud divisor)
  // system clock is 80 MHz
  UART0_IBRD_R = 520; // Integer part of BRD = 80,000,000 / (16 * 9600) = 520.83 (approximated to 520)
  UART0_FBRD_R = 53; // Fractional part of BRD = Round(0.83 * 64) = 53

  // Configure line control settings: 8-bit data, 1 stop bit, no parity
  UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);

  // Enable FIFO and set RX FIFO level to 1/8
  UART0_IFLS_R &= ~UART_IFLS_RX_M;
  UART0_IFLS_R |= UART_IFLS_RX1_8;

  // Enable UART0, RX, and TX
  UART0_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);
}

// Transmit a character via UART0
void UART0_SendChar(char data) {
  // Wait until UART is ready for transmission
  while (UART0_FR_R & UART_FR_TXFF);

  // Transmit data
  UART0_DR_R = data;
}

// Transmit a string via UART0
void UART0_SendString(const char *data) {
  // Loop until end of string
  while (*data != '\0') {
    UART0_SendChar(*data);
    data++;
  }
}

void setup() {
  // Initialize UART0
  UART0_Init();

  // Register configurations
  // Enable the GPIO port for Flame Sensor (Port B)
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable clock for Port B
  while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {}; // Wait until Port B is ready
  GPIO_PORTB_DIR_R &= ~0x20; // Set PB5 as input
  GPIO_PORTB_AFSEL_R |= 0x20; // Enable alternate function on PB5
  GPIO_PORTB_DEN_R &= ~0x20; // Disable digital I/O on PB5
  GPIO_PORTB_AMSEL_R |= 0x20; // Enable analog function on PB5
  
  // Enable the ADC module
  SYSCTL_RCGCADC_R |= 0x01; // Enable ADC0
  while ((SYSCTL_PRADC_R & 0x01) == 0) {}; // Wait until ADC0 is ready
  ADC0_ACTSS_R &= ~0x08; // Disable SS3 during configuration
  ADC0_EMUX_R &= ~0xF000; // Software trigger conversion
  ADC0_SSMUX3_R = 0x0B; // Get input from channel 11 (AIN11, which is PB5)
  ADC0_SSCTL3_R = 0x06; // Take one sample at a time, set flag at 1st sample
  ADC0_ACTSS_R |= 0x08; // Enable ADC0 SS3
  
  // Enable the GPIO port for Buzzer (Port A)
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // Enable clock for Port A
  while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) == 0) {}; // Wait until Port A is ready
  GPIO_PORTA_DIR_R |= 0x04; // Set PA2 as output
  GPIO_PORTA_DEN_R |= 0x04; // Enable digital I/O on PA2
  
  // Enable the GPIO port for ESP32 Trigger (Port E)
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // Enable clock for Port E
  while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {}; // Wait until Port E is ready
  GPIO_PORTE_DIR_R |= 0x01; // Set PE0 as output
  GPIO_PORTE_DEN_R |= 0x01; // Enable digital I/O on PE0

  // Set initial states
  GPIO_PORTA_DATA_R |= 0x04; // Ensure buzzer is off initially
  GPIO_PORTE_DATA_R &= ~0x01; // Ensure ESP32 trigger is off initially
}

int readFlameSensor() {
  ADC0_PSSI_R |= 0x08; // Start a conversion sequence 3
  while ((ADC0_RIS_R & 0x08) == 0) {}; // Wait for conversion to complete
  int result = ADC0_SSFIFO3_R & 0xFFF; // Read the result (12-bit value)
  ADC0_ISC_R = 0x08; // Clear the completion flag
  return result;
}

void loop() {
  // Read the analog value from the flame sensor
  int flameValue = readFlameSensor();
  
  // Debugging: Print flame sensor value
  char buffer[30];
  sprintf(buffer, "Flame Sensor Value: %d\n", flameValue);
  UART0_SendString(buffer);

  // Check if flame is detected
  if (flameValue < flameThreshold) {
    if (!fireDetected) {
      // Fire just detected, record the start time
      fireStartTime = millis();
      fireDetected = true;
    }

    // Check if fire has been detected for longer than the threshold time
    if (!espTriggered && (millis() - fireStartTime >= detectionThresholdTime)) {
      UART0_SendString("Fire detected for more than 5 seconds! Ringing buzzer and triggering ESP32.\n");
      
      // Trigger ESP
      GPIO_PORTE_DATA_R |= 0x01; // Set PE0 high
      delay(100);  // Short delay to ensure signal is recognized by ESP32
      GPIO_PORTE_DATA_R &= ~0x01; // Set PE0 low
      
      // Mark ESP32 as triggered
      espTriggered = true;
      
      // Turn on the buzzer
      GPIO_PORTA_DATA_R &= ~0x04; // Set PA2 low
    }
  } else {
    // No fire detected
    fireDetected = false;
    espTriggered = false; // Reset ESP32 trigger flag

    // Turn off the buzzer
    GPIO_PORTA_DATA_R |= 0x04; // Set PA2 high
  }
 
  // Wait a short period before checking again
  delay(100);
}