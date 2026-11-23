#include "sensors.h"
#include "soft_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define AHT10_ADDR      0x38
#define AP3216C_ADDR    0x1E
static uint8_t icm_addr = 0x68;

void Sensors_ScanBus(char *out_buf, int buf_size) {
    int pos = snprintf(out_buf, buf_size, "I2C Scan found:");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (SoftI2C_Probe(addr)) {
            pos += snprintf(out_buf + pos, buf_size - pos, " 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        snprintf(out_buf + pos, buf_size - pos, " NONE (check SCL/SDA pins)");
    }
    snprintf(out_buf + pos, buf_size - pos, "\r\n");
}

void Sensors_InitAll(void) {
    SoftI2C_Init();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 1. Soft-reset and initialize AHT10 / AHT20 (0x38) on primary I2C bus */
    uint8_t aht_rst = 0xBA;
    SoftI2C_WriteBytes(AHT10_ADDR, &aht_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t aht20_init[3] = {0xBE, 0x08, 0x00};
    SoftI2C_WriteBytes(AHT10_ADDR, aht20_init, 3);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t aht10_init[3] = {0xE1, 0x08, 0x00};
    SoftI2C_WriteBytes(AHT10_ADDR, aht10_init, 3);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. Initialize AP3216C (Reset then ALS+PS+IR continuous mode = 0x03) */
    SoftI2C_WriteReg(AP3216C_ADDR, 0x00, 0x04);
    vTaskDelay(pdMS_TO_TICKS(20));
    SoftI2C_WriteReg(AP3216C_ADDR, 0x00, 0x03);

    /* 3. Initialize ICM20608 (Detect 0x68 or 0x69, wake from sleep) */
    if (SoftI2C_Probe(0x69)) icm_addr = 0x69;
    else                     icm_addr = 0x68;

    SoftI2C_WriteReg(icm_addr, 0x6B, 0x80); /* Device reset */
    vTaskDelay(pdMS_TO_TICKS(50));
    SoftI2C_WriteReg(icm_addr, 0x6B, 0x01); /* Auto-select PLL clock */
    SoftI2C_WriteReg(icm_addr, 0x6C, 0x00); /* Enable Accel & Gyro */
    SoftI2C_WriteReg(icm_addr, 0x1B, 0x00); /* Gyro +-250 dps */
    SoftI2C_WriteReg(icm_addr, 0x1C, 0x00); /* Accel +-2g */
}

/* Helper to bit-bang AHT10/AHT20 on PE0 (SCL) / PE1 (SDA) if wired to secondary bus */
static bool read_aht_on_pe(float *temp_c, float *hum_pct) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &g);

    #define PE_SCL_H() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET)
    #define PE_SCL_L() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)
    #define PE_SDA_H() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET)
    #define PE_SDA_L() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET)
    #define PE_SDA_R() HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
    #define PE_DLY()   for (volatile int d = 0; d < 40; d++) __NOP()

    PE_SCL_H(); PE_SDA_H(); PE_DLY();

    /* Send 0xBE init on PE0/PE1 once so AHT20 starts converting */
    static bool pe_inited = false;
    if (!pe_inited) {
        pe_inited = true;
        PE_SDA_L(); PE_DLY(); PE_SCL_L(); PE_DLY();
        uint8_t init_cmd[4] = {(0x38 << 1), 0xBE, 0x08, 0x00};
        for (int b = 0; b < 4; b++) {
            uint8_t byte = init_cmd[b];
            for (int i = 0; i < 8; i++) {
                if (byte & 0x80) PE_SDA_H(); else PE_SDA_L();
                PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SCL_L(); PE_DLY();
                byte <<= 1;
            }
            PE_SDA_H(); PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SCL_L(); PE_DLY();
        }
        PE_SDA_L(); PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SDA_H();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Start + write 0x38<<1 | 0 */
    PE_SDA_L(); PE_DLY(); PE_SCL_L(); PE_DLY();
    uint8_t cmd[4] = {(0x38 << 1), 0xAC, 0x33, 0x00};
    for (int b = 0; b < 4; b++) {
        uint8_t byte = cmd[b];
        for (int i = 0; i < 8; i++) {
            if (byte & 0x80) PE_SDA_H(); else PE_SDA_L();
            PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SCL_L(); PE_DLY();
            byte <<= 1;
        }
        PE_SDA_H(); PE_DLY(); PE_SCL_H(); PE_DLY();
        bool ack = (PE_SDA_R() == 0);
        PE_SCL_L(); PE_DLY();
        if (!ack) { PE_SDA_L(); PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SDA_H(); return false; }
    }
    PE_SDA_L(); PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SDA_H(); PE_DLY();

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Start + read 6 bytes from (0x38<<1 | 1) */
    PE_SDA_L(); PE_DLY(); PE_SCL_L(); PE_DLY();
    uint8_t raddr = (0x38 << 1) | 1;
    for (int i = 0; i < 8; i++) {
        if (raddr & 0x80) PE_SDA_H(); else PE_SDA_L();
        PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SCL_L(); PE_DLY();
        raddr <<= 1;
    }
    PE_SDA_H(); PE_DLY(); PE_SCL_H(); PE_DLY();
    if (PE_SDA_R() != 0) { PE_SCL_L(); PE_SDA_L(); PE_SCL_H(); PE_SDA_H(); return false; }
    PE_SCL_L(); PE_DLY();

    uint8_t raw[6] = {0};
    for (int idx = 0; idx < 6; idx++) {
        uint8_t val = 0;
        PE_SDA_H();
        for (int i = 0; i < 8; i++) {
            val <<= 1;
            PE_SCL_H(); PE_DLY();
            if (PE_SDA_R()) val |= 1;
            PE_SCL_L(); PE_DLY();
        }
        raw[idx] = val;
        if (idx < 5) PE_SDA_L(); else PE_SDA_H();
        PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SCL_L(); PE_DLY(); PE_SDA_H();
    }
    PE_SDA_L(); PE_DLY(); PE_SCL_H(); PE_DLY(); PE_SDA_H();

    uint32_t raw_hum = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | (raw[3] >> 4);
    uint32_t raw_tmp = (((uint32_t)raw[3] & 0x0F) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
    if (raw_hum == 0 && raw_tmp == 0) return false;

    *hum_pct = ((float)raw_hum * 100.0f) / 1048576.0f;
    *temp_c  = (((float)raw_tmp * 200.0f) / 1048576.0f) - 50.0f;
    return true;
}

void Sensors_ReadAll(StorageSensors_t *data) {
    memset(data, 0, sizeof(StorageSensors_t));

    /* --- 1. Read AHT10/AHT20 (Try PF1/PF0 first, fallback to PE0/PE1) --- */
    uint8_t trig[3] = {0xAC, 0x33, 0x00};
    if (SoftI2C_WriteBytes(AHT10_ADDR, trig, 3)) {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t raw[6] = {0};
        if (SoftI2C_ReadBytes(AHT10_ADDR, raw, 6)) {
            uint32_t raw_hum = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | (raw[3] >> 4);
            uint32_t raw_tmp = (((uint32_t)raw[3] & 0x0F) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
            if (raw_hum != 0 || raw_tmp != 0) {
                data->humidity_pct  = ((float)raw_hum * 100.0f) / 1048576.0f;
                data->temperature_c = (((float)raw_tmp * 200.0f) / 1048576.0f) - 50.0f;
                data->aht10_ok = true;
            }
        }
    }
    if (!data->aht10_ok) {
        data->aht10_ok = read_aht_on_pe(&data->temperature_c, &data->humidity_pct);
    }

    /* --- 2. Read AP3216C (ALS: 0x0C-0x0D, PS: 0x0E-0x0F) --- */
    uint8_t ap_buf[4] = {0};
    if (SoftI2C_ReadRegs(AP3216C_ADDR, 0x0C, ap_buf, 4)) {
        data->ambient_light = ((uint16_t)ap_buf[1] << 8) | ap_buf[0];
        data->proximity     = (((uint16_t)(ap_buf[3] & 0x3F)) << 4) | (ap_buf[2] & 0x0F);
        data->ap3216c_ok    = true;
    }

    /* --- 3. Read ICM20608 (Accel 0x3B-0x40, Temp 0x41-0x42, Gyro 0x43-0x48) --- */
    uint8_t icm_buf[14] = {0};
    if (SoftI2C_ReadRegs(icm_addr, 0x3B, icm_buf, 14)) {
        int16_t ax = (int16_t)((icm_buf[0] << 8) | icm_buf[1]);
        int16_t ay = (int16_t)((icm_buf[2] << 8) | icm_buf[3]);
        int16_t az = (int16_t)((icm_buf[4] << 8) | icm_buf[5]);
        int16_t gx = (int16_t)((icm_buf[8] << 8) | icm_buf[9]);
        int16_t gy = (int16_t)((icm_buf[10] << 8) | icm_buf[11]);
        int16_t gz = (int16_t)((icm_buf[12] << 8) | icm_buf[13]);

        data->accel_x_g  = ax / 16384.0f;
        data->accel_y_g  = ay / 16384.0f;
        data->accel_z_g  = az / 16384.0f;
        data->gyro_x_dps = gx / 131.0f;
        data->gyro_y_dps = gy / 131.0f;
        data->gyro_z_dps = gz / 131.0f;
        data->icm20608_ok = true;
    }
}