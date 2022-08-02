
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void ble_midi_init(void);
void MiDi_Send(uint8_t *notify_data, uint8_t l);
bool ble_midi_state(void);

#ifdef __cplusplus
}
#endif
