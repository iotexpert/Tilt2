#pragma once

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "wiced_bt_ble.h"
#include "GUI.h"

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

/////////////// Generally callable threadsafe - non blocking
char *tdm_getColorString(tdm_tiltHandle_t handle);       // Return a char * to the color string for the tilt handle
int tdm_getNumTilt();                                 // Returns the number of possible tilts (probably always 8)
uint32_t tdm_getActiveTiltMask();                     // Return a bitmask of the active handles
uint32_t tdm_getNumDataSeen(tdm_tiltHandle_t handle); // Return number of data points seen
tdm_tiltData_t *tdm_getTiltData(tdm_tiltHandle_t handle);
GUI_COLOR tdm_colorGUI(tdm_tiltHandle_t handle);

