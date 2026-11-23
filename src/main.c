#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "edge_ai.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

UART_HandleTypeDef huart1;

static SemaphoreHandle_t xStateMutex;
static StorageSensors_t g_sensors;
static EdgeAI_Result_t  g_ai_result;

/* System Operating Modes */
typedef enum {
    MODE_GUARD_ACTIVE = 0,   /* Normal consumer Edge AI monitoring */
    MODE_DATA_LOGGING = 1    /* CSV dataset collection mode for ML training */
} SystemMode_t;

static volatile SystemMode_t g_sys_mode = MODE_GUARD_ACTIVE;
static volatile uint8_t      g_log_label = 0;      /* 0..3 label during CSV logging */
static volatile bool         g_alarm_muted = false;
static volatile uint8_t      g_uart_cmd_flags = 0; /* Set immediately by USART1 RX ISR */
static StorageState_t        g_last_alert_state = STATE_SAFE_STORAGE;

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void Print_Board_Audit(void);

/* Hardware Interrupt for USART1 RX so Serial Monitor keys ('0'..'3') never get missed */
void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        uint8_t ch = (uint8_t)(huart1.Instance->DR & 0xFFU);
        if (ch == '0' || ch == 'm' || ch == 'M') g_uart_cmd_flags |= 0x01;
        if (ch == '1' || ch == 'l' || ch == 'L') g_uart_cmd_flags |= 0x02;
        if (ch == '2' || ch == 'a' || ch == 'A') g_uart_cmd_flags |= 0x04;
        if (ch == '3' || ch == 'n' || ch == 'N') g_uart_cmd_flags |= 0x08;
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

/* Prevent HAL_Init() from starting SysTick before FreeRTOS scheduler starts */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    (void)TickPriority;
    return HAL_OK;
}

uint32_t HAL_GetTick(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        return (uint32_t)xTaskGetTickCount();
    }
    return 0;
}

void vApplicationTickHook(void) {}

/* Read onboard RT-Spark buttons (PC0=KEY0, PC1=KEY1, PC4=KEY2, PC5=WK_UP - active low) */
static uint8_t Read_Buttons(void) {
    uint8_t mask = 0;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) mask |= 0x01; /* KEY0: Mute/Ack */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) mask |= 0x02; /* KEY1: Toggle CSV Log */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) mask |= 0x04; /* KEY2: Board Audit */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) mask |= 0x08; /* WK_UP: Cycle Log Label */
    return mask;
}

/* Task 1: Sensor Acquisition, Fault Check & Edge AI Inference (Priority 3 - 250 ms) */
void vSensorAITask(void *pvParameters) {
    (void)pvParameters;
    StorageSensors_t local_s = {0};
    EdgeAI_Result_t local_res = {0};

    Sensors_InitAll();
    EdgeAI_Init();
    Print_Board_Audit();

    for (;;) {
        Sensors_ReadAll(&local_s);

        /* Fault recovery: re-init I2C bus if all sensors fail simultaneously */
        if (!local_s.aht10_ok && !local_s.ap3216c_ok && !local_s.icm20608_ok) {
            Sensors_InitAll();
            Sensors_ReadAll(&local_s);
        }

        EdgeAI_Predict(&local_s, &local_res);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensors   = local_s;
            g_ai_result = local_res;

            if (local_res.predicted_class != STATE_SAFE_STORAGE &&
                local_res.predicted_class != g_last_alert_state) {
                g_alarm_muted = false;
            }
            g_last_alert_state = local_res.predicted_class;
            xSemaphoreGive(xStateMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* Task 2: Real-Time Consumer Feedback (LEDs, Buzzer, Physical Buttons & Serial Commands - 100 ms) */
void vAlertUITask(void *pvParameters) {
    (void)pvParameters;
    uint32_t phase = 0;
    uint8_t prev_btn = 0;
    char msg[128];

    for (;;) {
        /* 1. Combine Physical Push-Buttons + Interrupt-Driven Serial Keystrokes */
        uint8_t btn = Read_Buttons();
        uint8_t pressed = (btn & (~prev_btn)) | g_uart_cmd_flags;
        g_uart_cmd_flags = 0;
        prev_btn = btn;

        if (pressed & 0x01) { /* KEY0 or '0': Mute / Acknowledge Alarm */
            g_alarm_muted = !g_alarm_muted;
            int n = snprintf(msg, sizeof(msg), "\r\n[UI] Alarm Buzzer %s by User\r\n",
                             g_alarm_muted ? "MUTED" : "UNMUTED");
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, HAL_MAX_DELAY);
        }
        if (pressed & 0x02) { /* KEY1 or '1': Toggle Guard Mode <-> CSV Data Collection Mode */
            g_sys_mode = (g_sys_mode == MODE_GUARD_ACTIVE) ? MODE_DATA_LOGGING : MODE_GUARD_ACTIVE;
            if (g_sys_mode == MODE_DATA_LOGGING) {
                const char *hdr = "\r\n=== CSV DATA LOGGING MODE (Label=0..3 via '3' + Enter) ===\r\ntimestamp_ms,temp_c,humidity_pct,als,ps,ax,ay,az,gx,gy,gz,label\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t *)hdr, strlen(hdr), HAL_MAX_DELAY);
            } else {
                const char *ghdr = "\r\n=== GUARD INFERENCE MODE ACTIVE ===\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t *)ghdr, strlen(ghdr), HAL_MAX_DELAY);
            }
        }
        if (pressed & 0x04) { /* KEY2 or '2': Run On-Demand Board Audit & Diagnostics */
            Print_Board_Audit();
        }
        if (pressed & 0x08) { /* WK_UP or '3': Cycle Dataset Ground-Truth Label (0..3) */
            g_log_label = (g_log_label + 1) % STATE_COUNT;
            int n = snprintf(msg, sizeof(msg), "\r\n[LOG] Active Ground-Truth Label set to %u (%s)\r\n",
                             g_log_label, EdgeAI_GetClassName((StorageState_t)g_log_label));
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, HAL_MAX_DELAY);
        }

        /* 2. Drive Visual & Audible Consumer Feedback */
        StorageState_t current_state = STATE_SAFE_STORAGE;
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            current_state = g_ai_result.predicted_class;
            xSemaphoreGive(xStateMutex);
        }

        phase++;
        if (g_sys_mode == MODE_DATA_LOGGING) {
            GPIO_PinState rec_led = ((phase % 10) == 0 || (phase % 10) == 2) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10, rec_led);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, rec_led);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_8, GPIO_PIN_RESET);
        } else if (current_state == STATE_SAFE_STORAGE) {
            GPIO_PinState led = ((phase % 10) == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10, led);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, led);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_8, GPIO_PIN_RESET);
        } else {
            GPIO_PinState strobe = (phase & 1) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10, strobe);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, strobe);

            GPIO_PinState buzz = (!g_alarm_muted && ((phase % 10) < 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_8, buzz);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Task 3: Telemetry & Dataset CSV Streamer (Priority 1 - 250 ms / 1000 ms) */
void vTelemetryTask(void *pvParameters) {
    (void)pvParameters;
    char buf[256];
    StorageSensors_t s = {0};
    EdgeAI_Result_t res = {0};

    vTaskDelay(pdMS_TO_TICKS(600));

    for (;;) {
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s   = g_sensors;
            res = g_ai_result;
            xSemaphoreGive(xStateMutex);
        }

        int t10   = (int)(s.temperature_c * 10.0f);
        int h10   = (int)(s.humidity_pct * 10.0f);
        int ax100 = (int)(s.accel_x_g * 100.0f);
        int ay100 = (int)(s.accel_y_g * 100.0f);
        int az100 = (int)(s.accel_z_g * 100.0f);
        int gx10  = (int)(s.gyro_x_dps * 10.0f);
        int gy10  = (int)(s.gyro_y_dps * 10.0f);
        int gz10  = (int)(s.gyro_z_dps * 10.0f);

        if (g_sys_mode == MODE_DATA_LOGGING) {
            int len = snprintf(buf, sizeof(buf),
                "%lu,%d.%d,%d.%d,%u,%u,%d.%02d,%d.%02d,%d.%02d,%d.%d,%d.%d,%d.%d,%u\r\n",
                (unsigned long)xTaskGetTickCount(),
                t10 / 10, abs(t10 % 10),
                h10 / 10, abs(h10 % 10),
                s.ambient_light, s.proximity,
                ax100 / 100, abs(ax100 % 100),
                ay100 / 100, abs(ay100 % 100),
                az100 / 100, abs(az100 % 100),
                gx10 / 10, abs(gx10 % 10),
                gy10 / 10, abs(gy10 % 10),
                gz10 / 10, abs(gz10 % 10),
                g_log_label);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(250)); /* 4 Hz dataset sampling */
        } else {
            int conf = (int)(res.confidence_pct);
            int len = snprintf(buf, sizeof(buf),
                "[%lu ms] T:%d.%dC H:%d.%d%% | ALS:%u PS:%u | Acc(%d,%d,%d) => AI:[%s] (%d%%) | Lat:%lu us (%lu cyc)%s\r\n",
                (unsigned long)xTaskGetTickCount(),
                t10 / 10, abs(t10 % 10),
                h10 / 10, abs(h10 % 10),
                s.ambient_light, s.proximity,
                ax100, ay100, az100,
                EdgeAI_GetClassName(res.predicted_class),
                conf,
                (unsigned long)res.inference_us,
                (unsigned long)res.inference_cycles,
                g_alarm_muted ? " [MUTED]" : "");
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void Print_Board_Audit(void) {
    char buf[420];
    char scan[128];
    Sensors_ScanBus(scan, sizeof(scan));

    StorageSensors_t s;
    Sensors_ReadAll(&s);

    int len = snprintf(buf, sizeof(buf),
        "\r\n--- RT-SPARK BOARD AUDIT & DIAGNOSTICS ---\r\n"
        "%s"
        "Sensors: AHT20=%s | AP3216C=%s | ICM20608=%s\r\n"
        "FreeRTOS Free Heap: %u bytes | SysClock: %lu MHz\r\n"
        "Type in Serial Monitor (+ Enter): [0]=Mute | [1]=CSV Log | [2]=Audit | [3]=Label\r\n"
        "------------------------------------------\r\n",
        scan,
        s.aht10_ok ? "OK" : "FAIL",
        s.ap3216c_ok ? "OK" : "FAIL",
        s.icm20608_ok ? "OK" : "FAIL",
        (unsigned int)xPortGetFreeHeapSize(),
        (unsigned long)(SystemCoreClock / 1000000U));
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

int main(void) {
    /* Enable Cortex-M4F Hardware Floating-Point Unit */
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
    __DSB();
    __ISB();

    HAL_Init();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    const char *boot_msg = "\r\n=== Smart Medicine & Pantry Storage Guard (Edge AI + FreeRTOS) ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)boot_msg, strlen(boot_msg), HAL_MAX_DELAY);

    xStateMutex = xSemaphoreCreateMutex();

    xTaskCreate(vSensorAITask,  "SensorAI",  768, NULL, 3, NULL);
    xTaskCreate(vAlertUITask,   "AlertUI",   384, NULL, 2, NULL);
    xTaskCreate(vTelemetryTask, "Telemetry", 768, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) {}
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Onboard LEDs */
    GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Onboard Buzzer pins */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* RT-Spark Onboard User Push-Buttons: PC0 (KEY0), PC1 (KEY1), PC4 (KEY2), PC5 (WK_UP) */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Enable RXNE Interrupt on USART1 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}