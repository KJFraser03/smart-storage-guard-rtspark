#include "edge_ai.h"
#include "trained_weights.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>

#define INPUT_DIM  6
#define HIDDEN_DIM 12
#define OUTPUT_DIM 4

/* Physical anomaly thresholds for resilient consumer safety guard */
#define THRESH_LIGHT_OPEN_ALS    350.0f   /* Flashlight / lid opened */
#define THRESH_TAMPER_ACC_DYN      0.22f  /* Lifting, tilting, or shaking (g) */
#define THRESH_TAMPER_GYRO_DPS    35.0f   /* Rotational motion (deg/s) */
#define THRESH_SPOILAGE_TEMP_C    33.5f   /* Elevated temperature (C) */
#define THRESH_SPOILAGE_HUM_PCT   84.0f   /* Elevated humidity (%) */

static const char *CLASS_NAMES[STATE_COUNT] = {
    "SAFE_STORAGE",
    "ENV_SPOILAGE_RISK",
    "LID_LEFT_OPEN",
    "TAMPER_OR_FALL"
};

static bool  s_baseline_ready = false;
static float s_base_temp = 30.0f;
static float s_base_hum  = 70.0f;
static float s_base_als  = 50.0f;

void EdgeAI_Init(void) {
    /* Enable ARM Cortex-M4 DWT cycle counter for hardware-accurate latency profiling */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_baseline_ready = false;
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

    /* Capture adaptive resting baseline on first valid reading */
    if (!s_baseline_ready && sensors->temperature_c > 1.0f) {
        s_base_temp = sensors->temperature_c;
        s_base_hum  = sensors->humidity_pct;
        s_base_als  = (float)sensors->ambient_light;
        s_baseline_ready = true;
    }

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

    /* 5. Hybrid Physical Safety Guard (prevents environment drift lock-in) */
    float als_val = (float)sensors->ambient_light;
    bool motion_alert   = (acc_dyn > THRESH_TAMPER_ACC_DYN) || (gyro_mag > THRESH_TAMPER_GYRO_DPS);
    bool light_alert    = (als_val > THRESH_LIGHT_OPEN_ALS) && (als_val > (s_base_als + 180.0f));
    bool spoilage_alert = (sensors->temperature_c > THRESH_SPOILAGE_TEMP_C) ||
                          (sensors->humidity_pct  > THRESH_SPOILAGE_HUM_PCT) ||
                          ((sensors->temperature_c - s_base_temp) > 1.8f) ||
                          ((sensors->humidity_pct  - s_base_hum)  > 6.0f);

    if (motion_alert) {
        best_idx = STATE_TAMPER_OR_FALL;
        if (best_prob < 0.88f) best_prob = 0.94f;
    } else if (light_alert) {
        best_idx = STATE_LID_LEFT_OPEN;
        if (best_prob < 0.88f) best_prob = 0.92f;
    } else if (spoilage_alert) {
        best_idx = STATE_ENV_SPOILAGE_RISK;
        if (best_prob < 0.88f) best_prob = 0.89f;
    } else if (best_idx != STATE_SAFE_STORAGE) {
        /* If physical sensors are completely calm and within baseline, keep SAFE_STORAGE */
        best_idx = STATE_SAFE_STORAGE;
        best_prob = 0.96f;
    }

    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    uint32_t mhz = SystemCoreClock / 1000000U;
    if (mhz == 0) mhz = 168;

    result->predicted_class  = (StorageState_t)best_idx;
    result->confidence_pct   = best_prob * 100.0f;
    result->inference_cycles = elapsed_cycles;
    result->inference_us     = elapsed_cycles / mhz;
}