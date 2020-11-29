#include <stdio.h>
#include <stdlib.h>

#include "cybsp.h"

#include "FreeRTOS.h"

#include "bluetoothManager.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "btutil.h"
#include "tiltDataManager.h"

/**************************************************************************************************
* Function Name: app_bt_management_callback()
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
				wiced_bt_ble_observe(WICED_TRUE, 0,tdm_processIbeacon);
            }
            else
            {
            	printf("Error enabling BTM_ENABLED_EVENT\n");
            }

            break;

        default:
            printf("Unhandled Bluetooth Management Event: %s\n", btutil_getBTEventName (event));
            break;
    }

    return result;
}
