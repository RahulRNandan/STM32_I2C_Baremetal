# I2C1 Peripheral Initialization and Communication on STM32

This project demonstrates the implementation of I2C communication using the I2C1 peripheral on an STM32 microcontroller. The program initializes I2C1, configures GPIO pins for I2C communication, and performs basic read and write operations to a slave device over the I2C bus.

### Table of Contents:
1. [Memory-mapped Registers](#memory-mapped-registers)
2. [Clock Setup](#clock-setup)
3. [GPIO Configuration](#gpio-configuration)
4. [I2C Initialization](#i2c-initialization)
5. [I2C Read/Write Functions](#i2c-readwrite-functions)
6. [Main Function](#main-function)
7. [License](#license)

## Memory-mapped Registers

The I2C peripheral registers and the GPIO registers are accessed directly using their memory addresses. These addresses are memory-mapped, which means they can be controlled via direct read/write operations.

```c
#define I2C1_BASE_ADDR  0x40005400  // Base address for I2C1 peripheral
#define RCC_BASE_ADDR   0x40023800  // Base address for RCC (Reset and Clock Control)
#define GPIOB_BASE_ADDR 0x40020400  // Base address for GPIOB (for I2C1 pins PB6 and PB7)
```

The `#define` directives map the base addresses of the I2C1 peripheral, the RCC (Reset and Clock Control) register, and the GPIOB register to specific memory locations. These are hardware-specific memory addresses in the STM32 microcontroller.

### Memory-mapped I2C1 Registers:
- **I2C1_CR1**: Control Register 1 (used to enable I2C and control the peripheral)
- **I2C1_CR2**: Control Register 2 (configures the peripheral frequency and other features)
- **I2C1_OAR1**: Own Address Register 1 (used to configure the device's I2C address in slave mode)
- **I2C1_DR**: Data Register (used for data transmission and reception)
- **I2C1_SR1**: Status Register 1 (provides information about the status of the I2C bus)
- **I2C1_SR2**: Status Register 2 (additional status information)
- **I2C1_CCR**: Clock Control Register (sets the I2C communication speed)
- **I2C1_TRISE**: TRISE Register (configures the maximum rise time)

### Memory-mapped RCC and GPIO Registers:
- **RCC_AHB1ENR**: AHB1 Peripheral Clock Enable Register (enables the clock for GPIOB)
- **RCC_APB1ENR**: APB1 Peripheral Clock Enable Register (enables the clock for I2C1)
- **GPIOB_MODER**: GPIO Mode Register (configures the mode for GPIO pins)
- **GPIOB_OTYPER**: GPIO Output Type Register (configures the output type for GPIO pins)
- **GPIOB_AFRL**: GPIO Alternate Function Low Register (sets the alternate function for GPIO pins)

---

## Clock Setup

Before configuring the I2C peripheral, the clock for both GPIOB (used for I2C pins) and I2C1 must be enabled.

```c
RCC_AHB1ENR |= (1 << 1);  // Enable GPIOB clock
RCC_APB1ENR |= (1 << 21); // Enable I2C1 clock
```
- `RCC_AHB1ENR` enables the clock for GPIOB, which is necessary for configuring the I2C pins (PB6 and PB7).
- `RCC_APB1ENR` enables the clock for I2C1, so it can be used for communication.

---

## GPIO Configuration

To configure the GPIO pins (PB6 for SCL and PB7 for SDA) for I2C functionality, they must be set to alternate function mode. Additionally, I2C requires open-drain outputs.

```c
GPIOB_MODER &= ~((3 << 12) | (3 << 14));  // Clear PB6, PB7 mode bits
GPIOB_MODER |= (2 << 12) | (2 << 14);     // Set PB6, PB7 to alternate function mode
GPIOB_OTYPER |= (1 << 6) | (1 << 7);      // Set PB6, PB7 to open-drain mode
GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28)); // Clear AF for PB6, PB7
GPIOB_AFRL |= (4 << 24) | (4 << 28);      // Set AF4 (I2C1) for PB6, PB7
```
- **GPIOB_MODER** sets PB6 and PB7 to alternate function mode, required for I2C communication.
- **GPIOB_OTYPER** configures PB6 and PB7 as open-drain, a standard requirement for I2C.
- **GPIOB_AFRL** assigns alternate function `AF4` to PB6 and PB7, which corresponds to I2C1.

---

## I2C Initialization

The I2C1 peripheral is reset and configured to operate at 100 kHz. The `CCR` and `TRISE` registers are set based on the APB1 clock frequency (16 MHz in this case).

```c
I2C1_CR1 |= (1 << 15); // Set SWRST bit in CR1 (Reset I2C1)
I2C1_CR1 &= ~(1 << 15); // Clear SWRST bit (Release reset)

I2C1_CR2 = I2C_FREQUENCY; // Set APB1 frequency in MHz
I2C1_CCR = (I2C_FREQUENCY * 1000000) / (2 * I2C_SPEED); // Configure clock control
I2C1_TRISE = I2C_FREQUENCY + 1; // Configure rise time
I2C1_CR1 |= (1 << 0); // Enable I2C1 peripheral (PE bit)
```
- **I2C1_CR1** is used to reset and enable the I2C1 peripheral.
- **I2C1_CR2** configures the APB1 clock frequency (16 MHz in this case).
- **I2C1_CCR** sets the clock control for 100 kHz I2C speed.
- **I2C1_TRISE** ensures that the rise time meets I2C specifications.

---

## I2C Read/Write Functions

### Start Condition
The `i2c_start` function sends a start condition on the I2C bus.

```c
void i2c_start() {
    I2C1_CR1 |= (1 << 8);  // Set start condition (START bit)
    while (!(I2C1_SR1 & (1 << 0))); // Wait for SB bit (start condition generated)
}
```
- **I2C1_CR1** sets the start condition, and the program waits until the start condition is sent (indicated by the SB bit in **I2C1_SR1**).

### Stop Condition
The `i2c_stop` function sends a stop condition on the I2C bus.

```c
void i2c_stop() {
    I2C1_CR1 |= (1 << 9);  // Set stop condition (STOP bit)
}
```

### Write Function
The `i2c_write` function writes data to the I2C bus.

```c
void i2c_write(uint8_t data) {
    while (!(I2C1_SR1 & (1 << 7))); // Wait until Data Register Empty (TXE)
    I2C1_DR = data; // Write data to Data Register
    while (!(I2C1_SR1 & (1 << 2))); // Wait for BTF (Byte Transfer Finished)
}
```
- **I2C1_DR** holds the data to be transmitted, and **I2C1_SR1** is used to monitor the status of the transmission.

### Read Functions
- `i2c_read_ack()` reads a byte from the I2C bus and sends an acknowledgment (ACK).
- `i2c_read_nack()` reads a byte from the I2C bus and sends a non-acknowledgment (NACK).

```c
uint8_t i2c_read_ack() {
    I2C1_CR1 |= (1 << 10); // Enable ACK
    while (!(I2C1_SR1 & (1 << 6))); // Wait for RXNE
    return I2C1_DR; // Return received data
}

uint8_t i2c_read_nack() {
    I2C1_CR1 &= ~(1 << 10); // Disable ACK
    while (!(I2C1_SR1 & (1 << 6))); // Wait for RXNE
    return I2C1_DR; // Return received data
}
```

---

## Main Function

```c
int main() {
    i2c_init();  // Initialize I2C1

    // Writing to a slave device
    i2c_start();
    i2c_write(0xA0);  // Slave address (write mode)
    i2c_write(0x00);  // Memory address

 to write to
    i2c_write(0x55);  // Data to write
    i2c_stop();

    // Reading from a slave device
    i2c_start();
    i2c_write(0xA0);  // Slave address (write mode)
    i2c_write(0x00);  // Memory address to read from
    i2c_start();      // Repeated start
    i2c_write(0xA1);  // Slave address (read mode)
    uint8_t data = i2c_read_nack();
    i2c_stop();

    while (1);
}
```
- **i2c_start()**, **i2c_write()**, and **i2c_stop()** control the I2C communication.
- The program writes a value to a memory address of a slave device and then reads the value back.

---

## License

This project is licensed under the MIT License.

---
# STM32_I2C_Baremetal
