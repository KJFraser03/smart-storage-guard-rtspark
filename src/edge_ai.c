#include "edge_ai.h"
#include "trained_weights.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define INPUT_DIM  6
#define HIDDEN_DIM 12
#define OUTPUT_DIM 4

static const char *CLASS_NAMES[STATE_COUNT] = {
    "SAFE_STORAGE",
    "ENV_SPOILAGE_RISK",
    "LID_LEFT_OPEN",
    "TAMPER_OR_FALL"
};

void EdgeAI_Init(void) {
    /* Enable ARM Cortex-M4 DWT cycle counter for hardware-accurate latency profiling */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

const char *EdgeAI_GetClassName(StorageState_t state) {
    if (state >= STATE_COUNT) return "UNKNOWN";
    return CLASS_NAMES[state];
}

void EdgeAI_Predict(const StorageSensors_t *sensors, EdgeAI_Result_t *result) {
    uint32_t start_cycles = DWT->CYCCNT;

    /* 1. Signal Processing & Feature Extraction (6D) */
    float acc_mag = sqrtf(sensors->accel_x_g * sensors->accel_x_g +
                          sensors->accel_y_g * sensors->accel_y_g +
                          sensors->accel_z_g * sensors->accel_z_g);
    float acc_dyn = fabsf(acc_mag - 0.95f);

    float gyro_mag = sqrtf(sensors->gyro_x_dps * sensors->gyro_x_dps +
                           sensors->gyro_y_dps * sensors->gyro_y_dps +
                           sensors->gyro_z_dps * sensors->gyro_z_dps);

    float raw_x[INPUT_DIM] = {
        sensors->temperature_c,
        sensors->humidity_pct,
        (float)sensors->ambient_light,
        (float)sensors->proximity,
        acc_dyn,
        gyro_mag
    };

    /* StandardScaler normalization exported from Python */
    float x[INPUT_DIM];
    for (int i = 0; i < INPUT_DIM; i++) {
        float scale = (FEAT_SCALE[i] > 1e-6f) ? FEAT_SCALE[i] : 1.0f;
        x[i] = (raw_x[i] - FEAT_MEAN[i]) / scale;
    }

    /* 2. Hidden Layer (6 -> 12) with ReLU */
    float hidden[HIDDEN_DIM];
    for (int j = 0; j < HIDDEN_DIM; j++) {
        float sum = B1[j];
        for (int i = 0; i < INPUT_DIM; i++) {
            sum += W1[j][i] * x[i];
        }
        hidden[j] = (sum > 0.0f) ? sum : 0.0f;
    }

    /* 3. Output Layer (12 -> 4) */
    float logits[OUTPUT_DIM];
    float max_logit = -1e9f;
    for (int k = 0; k < OUTPUT_DIM; k++) {
        float sum = B2[k];
        for (int j = 0; j < HIDDEN_DIM; j++) {
            sum += W2[k][j] * hidden[j];
        }
        logits[k] = sum;
        if (sum > max_logit) {
            max_logit = sum;
        }
    }

    /* 4. Numerically Stable Softmax */
    float sum_exp = 0.0f;
    for (int k = 0; k < OUTPUT_DIM; k++) {
        result->probabilities[k] = expf(logits[k] - max_logit);
        sum_exp += result->probabilities[k];
    }

    int best_idx = 0;
    float best_prob = 0.0f;
    for (int k = 0; k < OUTPUT_DIM; k++) {
        result->probabilities[k] /= sum_exp;
        if (result->probabilities[k] > best_prob) {
            best_prob = result->probabilities[k];
            best_idx = k;
        }
    }

    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    uint32_t mhz = SystemCoreClock / 1000000U;
    if (mhz == 0) mhz = 168;

    result->predicted_class  = (StorageState_t)best_idx;
    result->confidence_pct   = best_prob * 100.0f;
    result->inference_cycles = elapsed_cycles;
    result->inference_us     = elapsed_cycles / mhz;
}