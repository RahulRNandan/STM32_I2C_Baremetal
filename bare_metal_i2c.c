#include <stdint.h>

#define I2C1_BASE_ADDR  0x40005400  // Base address for I2C1 peripheral
#define RCC_BASE_ADDR   0x40023800  // Base address for RCC (Reset and Clock Control)
#define GPIOB_BASE_ADDR 0x40020400  // Base address for GPIOB (for I2C1 pins PB6 and PB7)

// Memory-mapped I/O Registers for I2C1 Peripheral
#define I2C1_CR1   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x00))) // Control Register 1
#define I2C1_CR2   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x04))) // Control Register 2
#define I2C1_OAR1  (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x08))) // Own Address Register 1
#define I2C1_DR    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x10))) // Data Register
#define I2C1_SR1   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x14))) // Status Register 1
#define I2C1_SR2   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x18))) // Status Register 2
#define I2C1_CCR   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x1C))) // Clock Control Register
#define I2C1_TRISE (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x20))) // TRISE Register

// RCC Registers for Clock Control
#define RCC_AHB1ENR (*((volatile uint32_t *)(RCC_BASE_ADDR + 0x30))) // AHB1 Peripheral Clock Enable Register
#define RCC_APB1ENR (*((volatile uint32_t *)(RCC_BASE_ADDR + 0x40))) // APB1 Peripheral Clock Enable Register

// GPIOB Registers
#define GPIOB_MODER  (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x00))) // GPIOB Mode Register
#define GPIOB_OTYPER (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x04))) // GPIOB Output Type Register
#define GPIOB_AFRL   (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x20))) // GPIOB Alternate Function Low Register

// I2C Timing Constants
#define I2C_FREQUENCY 16 // 16 MHz APB1 clock
#define I2C_SPEED 100000 // 100 kHz I2C speed


// Function Prototypes
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

void i2c_init() {
    // Enable GPIOB clock (AHB1)
    RCC_AHB1ENR |= (1 << 1);  // Enable GPIOB clock
    
    // Enable I2C1 clock (APB1)
    RCC_APB1ENR |= (1 << 21); // Enable I2C1 clock

    // Configure GPIOB Pins (PB6 for SCL, PB7 for SDA) to alternate function
    GPIOB_MODER &= ~((3 << 12) | (3 << 14));  // Clear PB6, PB7 mode bits
    GPIOB_MODER |= (2 << 12) | (2 << 14);     // Set PB6, PB7 to alternate function mode
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);      // Set PB6, PB7 to open-drain mode
    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28)); // Clear AF for PB6, PB7
    GPIOB_AFRL |= (4 << 24) | (4 << 28);      // Set AF4 (I2C1) for PB6, PB7

    // Reset I2C1 peripheral
    I2C1_CR1 |= (1 << 15); // Set SWRST bit in CR1
    I2C1_CR1 &= ~(1 << 15); // Clear SWRST bit to release reset

    // Configure I2C1 peripheral
    I2C1_CR2 = I2C_FREQUENCY; // Set APB1 frequency in MHz
    I2C1_CCR = (I2C_FREQUENCY * 1000000) / (2 * I2C_SPEED); // Configure clock control for 100kHz I2C
    I2C1_TRISE = I2C_FREQUENCY + 1; // Maximum rise time for I2C1

    I2C1_CR1 |= (1 << 0); // Enable I2C1 peripheral (PE bit)
}

void i2c_start() {
    I2C1_CR1 |= (1 << 8);  // Set start condition (START bit)
    while (!(I2C1_SR1 & (1 << 0))); // Wait for SB bit (start condition generated)
}

void i2c_stop() {
    I2C1_CR1 |= (1 << 9);  // Set stop condition (STOP bit)
}

void i2c_write(uint8_t data) {
    while (!(I2C1_SR1 & (1 << 7))); // Wait until Data Register Empty (TXE)
    I2C1_DR = data; // Write data to Data Register
    while (!(I2C1_SR1 & (1 << 2))); // Wait for BTF (Byte Transfer Finished)
}

uint8_t i2c_read_ack() {
    I2C1_CR1 |= (1 << 10); // Enable ACK
    while (!(I2C1_SR1 & (1 << 6))); // Wait for RXNE (Receive Data Register Not Empty)
    return I2C1_DR; // Return received data
}

uint8_t i2c_read_nack() {
    I2C1_CR1 &= ~(1 << 10); // Disable ACK
    while (!(I2C1_SR1 & (1 << 6))); // Wait for RXNE (Receive Data Register Not Empty)
    return I2C1_DR; // Return received data
}

int main() {
    i2c_init();  // Initialize I2C1

    // Example: Writing to a slave device
    i2c_start();
    i2c_write(0xA0);  // Slave address (write mode)
    i2c_write(0x00);  // Memory address
    i2c_write(0x55);  // Data to write
    i2c_stop();

    // Example: Reading from a slave device
    i2c_start();
    i2c_write(0xA0);      // Slave address (write mode)
    i2c_write(0x00);      // Memory address
    i2c_start();          // Repeated start
    i2c_write(0xA1);      // Slave address (read mode)
    uint8_t data = i2c_read_nack();  // Read data with NACK
    i2c_stop();

    while (1) {
        // In the future, this loop could handle low-power modes, 
        // or you might add code here for periodic tasks.
    }
}
