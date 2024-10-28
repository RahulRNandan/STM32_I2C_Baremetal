#include <cstdint>   // standard C++ header for fixed width int type
#include <stdint.h> // C standard header for fixed width int type

// Define base addresses for I2C1, RCC, and GPIOB peripherals
#define I2C1_BASE_ADDR 0x40005400   // Base address of I2C1
#define RCC_BASE_ADDR 0x40023800    // Base address of RCC (Reset and Clock Control)
#define GPIOB_BASE_ADDR 0x40020400 // Base address of GPIOB

// I2C1 Registers (Memory-mapped I/O)
#define I2C1_CR1    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x00)))   // Control register 1
#define I2C1_CR2    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x04)))   // Control register 2
#define I2C1_OAR1   (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x08)))   // Own address register 1
#define I2C1_DR     (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x10)))   // Data register 1
#define I2C1_SR1    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x14)))   // Status register 1
#define I2C1_SR2    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x18)))   // Status register 2
#define I2C1_CCR    (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x1c)))   // Clock control register
#define I2C1_TRISE  (*((volatile uint32_t *)(I2C1_BASE_ADDR + 0x20)))   // TRISE register for max rise time

// RCC Registers for Clock Control
#define RCC_AHB1ENR  (*((volatile uint32_t *)(RCC_BASE_ADDR + 0x00)))   // AHB1 peripheral clock enable register
#define GPIOB_MODER  (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x00))) // GPIO port mode register
#define GPIOB_OTYPER (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x04))) // GPIO output type register
#define GPIOB_AFRL   (*((volatile uint32_t *)(GPIOB_BASE_ADDR + 0x20))) // GPIO alternate function low register


// I2C Timing Constants
#define I2C_FREQUENCY 16    // APB1 frequency in MHz (16 MHz)
#define I2C_SPEED 100000    // Standard I2C speed of 100kHz


class I2C{
public:
    // Constructor to initialize the I2C peripheral
    I2C(){
        init(); // Call the initialization function
    }

   // Function to send a START condition on the I2C bus
    void start(){
        I2C1_CR1 |= (1<<8); // Set the START bit in CR1
        while (!(I2C1_SR1 & (1 << 0))); // Wait until the START condition is generated (SB bit is set in SR1)
 
    }
    // Function to send a STOP condition on the I2C bus
    void stop(){
        I2C1_CR1 |= (1 << 9); // Set the STOP bit in CR1
    }
    // Function to write data to the I2C bus
    void write(uint8_t data){
        // Wait until the data register is empty (TXE bit is set in SR1)
        while(!(I2C1_SR1 & (1 << 7)));
        // Write the data to the data register
        I2C1_DR = data;
        // Wait until the byte transfer is finished (BTF bit is set in SR1)
        while (!(I2C1_SR1 & (1 << 2)));
    }
    // Function to read data from I2C bus and send an ACK
    uint8_t read_ack(){
        // Enable ACK by setting ACK bit in CR1
        I2C1_CR1 |= (1 << 10);
        // Wait until the data is received (RXNE bit is set in SR1)
        while (!(I2C1_SR1 & (1 << 6)));
        // return the Rx data from the data registers
        return I2C1_DR;
    }
    // Function to read data from the I2C bus and send an ACK
    uint8_t read_nack(){
        // Enable ACK by setting ACK bit in CR1
        I2C1_CR1 &= ~(1 << 10);
        // Wait until the data is received (RXNE bit is set in SR1)
        while(!(I2C1_SR1 & (1 << 6)));
        // Return the received data from the data register
        return I2C1_DR;
    }
private:
   void init () {
    // Enable GPIOB and I2C1 clocks
    RCC_AHB1ENR |= (1 << 1); // Enable GPIOB clock
    RCC_AHB1ENR |= (1 << 21); // Enable I2C1 clock

    // Configure GPIOB Pins (PB6 for SCL, PB7 for SDA) to alternate function
    GPIOB_MODER &= ~((3 << 12) | (3 << 14));  // Clear PB6, PB7 mode bits
    GPIOB_MODER |= (2 << 12) | (2 << 14);  // Set PB6, PB7 to alternate function mode
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);  // Set PB6, PB7 to open-drain mode
    
    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28));  // Clear AF for PB6, PB7
    GPIOB_AFRL |= (4 << 24) | (4 << 28);  // Set AF4 (I2C1) for PB6, PB7

    // Reset I2C1 peripheral
    I2C1_CR1 |= (1 << 15);  // Set SWRST bit in CR1
    I2C1_CR1 &= ~(1 << 15);  // Clear SWRST bit to release reset

    // Configure I2C1 peripheral
    I2C1_CR2 = I2C_FREQUENCY;  // Set APB1 frequency in MHz
    I2C1_CCR = (I2C_FREQUENCY * 1000000) / (2 * I2C_SPEED);  // Configure clock control for 100kHz I2C
    I2C1_TRISE = I2C_FREQUENCY + 1;  // Maximum rise time for I2C1       I2C1_CR1 |= (1 << 0);  // Enable I2C1 peripheral (PE bit)

   }
};

void vTaskFunction(void *pvParameters) {
    I2C i2c;
    // Example: Writing to a slave device
    i2c.start();            // Send START condition
    i2c.write(0xA0);   // Send slave funtion (write mode)
    i2c.write(0x00);   // Send mem addr/cmd
    i2c.write(0x55);   // Send STOP condition
    i2c.stop();

    // Example: Reading from a slave device
    i2c.start();              // send START condition
    i2c.write(0xA0);    // Send Slave address (write mode)
    i2c.start();             // Send repeated START condition
    i2c.write(0xA1);    // send slave address (read mode)
    uint8_t data = i2c.read_nack(); //Read data (send NACK)
    i2c.stop();             //  send STOP condition

    while (1) {
        // task loop
    }

}

int main(){
    return 0;
}