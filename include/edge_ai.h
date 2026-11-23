#ifndef EDGE_AI_H
#define EDGE_AI_H

#include "sensors.h"
#include <stdint.h>

typedef enum {
    STATE_SAFE_STORAGE = 0,
    STATE_ENV_SPOILAGE_RISK = 1,
    STATE_LID_LEFT_OPEN = 2,
    STATE_TAMPER_OR_FALL = 3,
    STATE_COUNT = 4
} StorageState_t;

typedef struct {
    StorageState_t predicted_class;
    float probabilities[STATE_COUNT];
    float confidence_pct;
    uint32_t inference_cycles;
    uint32_t inference_us;
} EdgeAI_Result_t;

void EdgeAI_Init(void);
void EdgeAI_Predict(const StorageSensors_t *sensors, EdgeAI_Result_t *result);
const char *EdgeAI_GetClassName(StorageState_t state);

#endif