#include "soft_i2c.h"

static void i2c_delay(void) {
    for (volatile int i = 0; i < 40; i++) {
        __NOP();
    }
}

static inline void scl_high(void) { HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET); }
static inline void scl_low(void)  { HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET); }
static inline void sda_high(void) { HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET); }
static inline void sda_low(void)  { HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET); }
static inline uint8_t sda_read(void) { return (uint8_t)HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN); }

void SoftI2C_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = I2C_SCL_PIN;
    HAL_GPIO_Init(I2C_SCL_PORT, &gpio);

    gpio.Pin = I2C_SDA_PIN;
    HAL_GPIO_Init(I2C_SDA_PORT, &gpio);

    scl_high();
    sda_high();
}

static void i2c_start(void) {
    sda_high(); scl_high(); i2c_delay();
    sda_low();  i2c_delay();
    scl_low();  i2c_delay();
}

static void i2c_stop(void) {
    sda_low();  i2c_delay();
    scl_high(); i2c_delay();
    sda_high(); i2c_delay();
}

static bool i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) sda_high();
        else             sda_low();
        i2c_delay();
        scl_high(); i2c_delay();
        scl_low();  i2c_delay();
        byte <<= 1;
    }
    sda_high(); i2c_delay();
    scl_high(); i2c_delay();
    bool ack = (sda_read() == 0);
    scl_low();  i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(bool send_ack) {
    uint8_t byte = 0;
    sda_high();
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high(); i2c_delay();
        if (sda_read()) byte |= 1;
        scl_low();  i2c_delay();
    }
    if (send_ack) sda_low();
    else          sda_high();
    i2c_delay();
    scl_high(); i2c_delay();
    scl_low();  i2c_delay();
    sda_high();
    return byte;
}

bool SoftI2C_Probe(uint8_t addr7) {
    i2c_start();
    bool ack = i2c_write_byte((uint8_t)(addr7 << 1));
    i2c_stop();
    return ack;
}

bool SoftI2C_WriteBytes(uint8_t addr7, const uint8_t *data, uint16_t len) {
    i2c_start();
    if (!i2c_write_byte((uint8_t)(addr7 << 1))) { i2c_stop(); return false; }
    for (uint16_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return false; }
    }
    i2c_stop();
    return true;
}

bool SoftI2C_ReadBytes(uint8_t addr7, uint8_t *data, uint16_t len) {
    i2c_start();
    if (!i2c_write_byte((uint8_t)((addr7 << 1) | 1))) { i2c_stop(); return false; }
    for (uint16_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i < (len - 1));
    }
    i2c_stop();
    return true;
}

bool SoftI2C_WriteReg(uint8_t addr7, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return SoftI2C_WriteBytes(addr7, buf, 2);
}

bool SoftI2C_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len) {
    if (!SoftI2C_WriteBytes(addr7, &reg, 1)) return false;
    return SoftI2C_ReadBytes(addr7, buf, len);
}