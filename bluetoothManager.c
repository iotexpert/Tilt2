#include <stdio.h>
#include <stdlib.h>

#include "cybsp.h"

#include "FreeRTOS.h"

#include "bluetoothManager.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"


#define TILT_IBEACON_DATA_LEN 20
typedef struct  {
    char *colorName;
    uint8_t uuid[TILT_IBEACON_DATA_LEN];
} tilt_t;

#define IBEACON_HEADER 0x4C,0x00,0x02,0x15

static tilt_t tiltDB [] =
{
    {"Red",    {IBEACON_HEADER,0xA4,0x95,0xBB,0x10,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Green" , {IBEACON_HEADER,0xA4,0x95,0xBB,0x20,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Black" , {IBEACON_HEADER,0xA4,0x95,0xBB,0x30,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Purple", {IBEACON_HEADER,0xA4,0x95,0xBB,0x40,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Orange", {IBEACON_HEADER,0xA4,0x95,0xBB,0x50,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Blue"  , {IBEACON_HEADER,0xA4,0x95,0xBB,0x60,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Yellow", {IBEACON_HEADER,0xA4,0x95,0xBB,0x70,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
    {"Pink"  , {IBEACON_HEADER,0xA4,0x95,0xBB,0x80,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE}},
};
#define NUM_TILT (sizeof(tiltDB)/sizeof(tilt_t))

static void btm_advScanResultCback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
	if (p_scan_result == 0)
		return;

	uint8_t mfgFieldLen;
	uint8_t *mfgFieldData;
	mfgFieldData = wiced_bt_ble_check_advertising_data(p_adv_data,BTM_BLE_ADVERT_TYPE_MANUFACTURER,&mfgFieldLen);
    
	if(mfgFieldData && mfgFieldLen == 25)
    {
        for(int i=0;i<NUM_TILT;i++)
        {
            if(memcmp(mfgFieldData,tiltDB[i].uuid,TILT_IBEACON_DATA_LEN)==0)
            {
                float gravity = ((float)((uint16_t)mfgFieldData[22] << 8 | (uint16_t)mfgFieldData[23]))/1000;
		        int temperature = mfgFieldData[20] << 8 | mfgFieldData[21];
                int8_t txPower = mfgFieldData[24];

                printf("Found Color=%s Gravity=%f Temperature = %d txPower=%d\n",tiltDB[i].colorName,gravity,temperature,txPower);
                break;
            }
        }
    }
}

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
				wiced_bt_ble_observe(WICED_TRUE, 0,btm_advScanResultCback);
            }
            else
            {
            	printf("Error enabling BTM_ENABLED_EVENT\n");
            }

            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x\n", event);
            break;
    }

    return result;
}
