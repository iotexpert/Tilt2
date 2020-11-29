#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiced_bt_ble.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "tiltDataManager.h"


/*********************************************************************************
*
* Tilt Database Definition
* 
*********************************************************************************/

#define TILT_IBEACON_HEADER_LEN 20
#define TILT_IBEACON_DATA_LEN 5
#define TILT_IBEACON_LEN (TILT_IBEACON_HEADER_LEN + TILT_IBEACON_DATA_LEN)
typedef struct  {
    char *colorName;
    uint8_t uuid[TILT_IBEACON_HEADER_LEN];
    tdm_tiltData_t *data;
    int numDataPoints;
    int numDataSeen;
} tilt_t;

#define IBEACON_HEADER 0x4C,0x00,0x02,0x15

static tilt_t tiltDB [] =
{
    {"Red",      {IBEACON_HEADER,0xA4,0x95,0xBB,0x10,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Green" ,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x20,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Black" ,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x30,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Purple",   {IBEACON_HEADER,0xA4,0x95,0xBB,0x40,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Orange",   {IBEACON_HEADER,0xA4,0x95,0xBB,0x50,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Blue"  ,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x60,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Yellow",   {IBEACON_HEADER,0xA4,0x95,0xBB,0x70,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Pink"  ,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x80,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
};
#define NUM_TILT (sizeof(tiltDB)/sizeof(tilt_t))

/*********************************************************************************
* 
* Tilt Data Manager External Interface Queue
*
*********************************************************************************/
typedef enum {
    TDM_CMD_ADD_DATAPOINT,
    TDM_CMD_PROCESS_DATA,
} tdm_cmd_t;

typedef struct {
    tdm_cmd_t cmd;
    tdm_tiltHandle_t tilt;
    void *msg;
} tdm_cmdMsg_t;

static QueueHandle_t tdm_cmdQueue;

static void tdm_submitProcessData();



// There is a bunch of data coming out of the tilts... every second on 3 channels
// So you may endup with a boatload of data
// This function will take an average of all of the data that has been saved

static void tdm_processData()
{
    for(int i=0;i<NUM_TILT;i++)
    {
        if(tiltDB[i].data == 0)
        {
            continue;
        }

        tiltDB[i].data->gravity /= tiltDB[i].numDataPoints;
        tiltDB[i].data->temperature /= tiltDB[i].numDataPoints;
        tiltDB[i].numDataPoints = 1;
        printf("Tilt %s Temperature = %d Gravity =%f\n",tiltDB[i].colorName,tiltDB[i].data->temperature,tiltDB[i].data->gravity);
    }
}


// This function will
// insert new data for that tilt if none has ever been seen
// or it will add the data to the current count
static void tdm_addData(tdm_tiltHandle_t handle, tdm_tiltData_t *data)
{
    if(tiltDB[handle].data == 0)
    {
        tiltDB[handle].data = data; 
        tiltDB[handle].numDataPoints = 1;
        tiltDB[handle].data->next=0;
    }
    else
    {
        tiltDB[handle].data->gravity     += data->gravity;
        tiltDB[handle].data->temperature += data->temperature;
        tiltDB[handle].numDataPoints += 1;

        free(data);
    }

    tiltDB[handle].numDataSeen += 1;
    tiltDB[handle].data->time    = data->time;
    tiltDB[handle].data->rssi    = data->rssi;
    tiltDB[handle].data->txPower = data->txPower;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Public Functions
// 
////////////////////////////////////////////////////////////////////////////////////////////////////////

void tdm_task(void *arg)
{
    tdm_cmdQueue = xQueueCreate(10,sizeof(tdm_cmdMsg_t));
    tdm_cmdMsg_t msg;

    TimerHandle_t tdm_processDataTimer;

    // Call the process data function once per hour to filter the input data
    tdm_processDataTimer=xTimerCreate("process",1000*30,pdTRUE,0,tdm_submitProcessData);
    xTimerStart(tdm_processDataTimer,0);

    while(1)
    {
        xQueueReceive(tdm_cmdQueue,&msg,portMAX_DELAY);
        switch(msg.cmd)
        {
            case TDM_CMD_ADD_DATAPOINT:
                tdm_addData(msg.tilt,msg.msg);
            break;

            case TDM_CMD_PROCESS_DATA:
                tdm_processData();
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// These functions submit commands to main command queue: tdm_cmdQueue
// 
////////////////////////////////////////////////////////////////////////////////////////////////////////

void tdm_submitNewData(tdm_tiltHandle_t handle,tdm_tiltData_t *data)
{
    tdm_cmdMsg_t msg;
    msg.msg = data;
    msg.tilt = handle;
    msg.cmd =  TDM_CMD_ADD_DATAPOINT;
    if(xQueueSend(tdm_cmdQueue,&msg,0) != pdTRUE)
    {
        printf("Failed to send to dmQueue\n");
        free(data);
    }
}

static void tdm_submitProcessData()
{
    tdm_cmdMsg_t msg;
    msg.cmd = TDM_CMD_PROCESS_DATA;
    xQueueSend(tdm_cmdQueue,&msg,0);
}


//
// This function is called in the Bluetooth Manager Context
// Specifically it is the advertising observer callback
//
void tdm_processIbeacon(wiced_bt_ble_scan_results_t *p_scan_result,uint8_t *p_adv_data)
{

	uint8_t mfgFieldLen;
	uint8_t *mfgAdvField;
	mfgAdvField = wiced_bt_ble_check_advertising_data(p_adv_data,BTM_BLE_ADVERT_TYPE_MANUFACTURER,&mfgFieldLen);
    
	if(!mfgAdvField)
        return;

    if(mfgFieldLen != TILT_IBEACON_LEN)
        return;

    for(int i=0;i<NUM_TILT;i++)
    {
        if(memcmp(mfgAdvField,tiltDB[i].uuid,TILT_IBEACON_HEADER_LEN) == 0)
        {
            uint32_t timerTime = xTaskGetTickCount() / 1000;
		    int8_t txPower = mfgAdvField[24];
		    float gravity = ((float)((uint16_t)mfgAdvField[22] << 8 | (uint16_t)mfgAdvField[23]))/1000;
		    int temperature = mfgAdvField[20] << 8 | mfgAdvField[21];

            // The tilt repeater will send out 0's if it hasnt heard anything ... and they send out some crazy stuff
            // when they first startup 
            if(gravity>2.0 || gravity<0.95 || temperature > 110 || gravity == 0 || temperature == 0)
                return;

            tdm_tiltData_t *data;
            data = malloc(sizeof(tdm_tiltData_t));
                
            data->gravity     = gravity;
            data->temperature = temperature;
            data->txPower     = txPower;
            data->time        = timerTime;
            data->rssi        = p_scan_result->rssi;

            tdm_submitNewData(i,data);
            
        }
    }
}
