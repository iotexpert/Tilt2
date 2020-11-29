#pragma once

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "wiced_bt_ble.h"

typedef struct {
    float gravity;
    int temperature;
    int8_t rssi;
    int8_t txPower;
	uint32_t time;
    struct tdm_tiltData_t *next;
} tdm_tiltData_t;

typedef int tdm_tiltHandle_t;

void tdm_task(void *arg);
void tdm_processIbeacon(wiced_bt_ble_scan_results_t *p_scan_result,uint8_t *p_adv_data);
