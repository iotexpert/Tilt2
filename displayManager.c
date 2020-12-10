#include <stdio.h>
#include <stdlib.h>

#include "GUI.h"
#include "mtb_st7789v.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "displayManager.h"
#include "tiltDataManager.h"

#include "IOTexpert_Logo_Vertical320x240.h"

const mtb_st7789v_pins_t tft_pins =
{
    .db08 = (CYBSP_J2_2),
    .db09 = (CYBSP_J2_4),
    .db10 = (CYBSP_J2_6),
    .db11 = (CYBSP_J2_10),
    .db12 = (CYBSP_J2_12),
    .db13 = (CYBSP_D7),
    .db14 = (CYBSP_D8),
    .db15 = (CYBSP_D9),
    .nrd  = (CYBSP_D10),
    .nwr  = (CYBSP_D11),
    .dc   = (CYBSP_D12),
    .rst  = (CYBSP_D13)
};

void dm_task(void *arg)
{

    /* Initialize the display controller */
    mtb_st7789v_init8(&tft_pins);
    GUI_Init();
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_DrawBitmap(&bmIOTexpert_Logo_Vertical320x240,0,11);

    while(1)
        vTaskDelay(1);
}


