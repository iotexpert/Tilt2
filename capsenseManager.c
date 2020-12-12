#include <stdio.h>

#include "cycfg_capsense.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "displayManager.h"

static SemaphoreHandle_t cpm_semaphore;

static void cpm_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

static void cpm_callback(cy_stc_active_scan_sns_t *ptrActiveScan)
{
    xSemaphoreGiveFromISR(cpm_semaphore,portMAX_DELAY);
}

void cpm_task()
{
    cpm_semaphore = xSemaphoreCreateCounting(10,0);

    static const cy_stc_sysint_t CapSense_ISR_cfg =
    {
        .intrSrc = csd_interrupt_IRQn, /* Interrupt source is the CSD interrupt */
        .intrPriority = 7u,            /* Interrupt priority is 7 */
    };

    Cy_CapSense_Init(&cy_capsense_context);
        
    Cy_SysInt_Init(&CapSense_ISR_cfg, &cpm_isr);
    NVIC_ClearPendingIRQ(CapSense_ISR_cfg.intrSrc);
    NVIC_EnableIRQ(CapSense_ISR_cfg.intrSrc);

    Cy_CapSense_RegisterCallback	(CY_CAPSENSE_END_OF_SCAN_E,cpm_callback, &cy_capsense_context); 
    Cy_CapSense_Enable (&cy_capsense_context);

    int button0Prev = 0;
    int button1Prev = 0;
    int button0Curr = 0;
    int button1Curr = 0;
    while(1)
    {
        Cy_CapSense_ScanAllWidgets (&cy_capsense_context);
        xSemaphoreTake(cpm_semaphore,portMAX_DELAY);
        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
        button0Curr = Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON0_WDGT_ID,&cy_capsense_context);
        button1Curr = Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON1_WDGT_ID,&cy_capsense_context);

        if(button0Curr != button0Prev && button0Curr == 1)
        {
            dm_submitAutoCmd();
        }
        if(button1Curr != button1Prev && button1Curr == 1)
        {
            dm_submitNextScreenCmd();
        }

        button0Prev = button0Curr;
        button1Prev = button1Curr;

        vTaskDelay(20); // 50 Hz Update Rate

    }
}