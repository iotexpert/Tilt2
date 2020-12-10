#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiced_bt_ble.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "tiltDataManager.h"
#include "gui.h"

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
    GUI_COLOR color;
    uint8_t uuid[20];
    tdm_tiltData_t *data;
    int numDataPoints;
    int numDataSeen;
} tilt_t;


#define IBEACON_HEADER 0x4C,0x00,0x02,0x15
#define GUI_PINK GUI_MAKE_COLOR(0x00CCCCFF)
#define GUI_PURPLE GUI_MAKE_COLOR(0x00800080)

static tilt_t tiltDB [] =
{
    {"Red",    GUI_RED,    {IBEACON_HEADER,0xA4,0x95,0xBB,0x10,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Green" , GUI_GREEN,  {IBEACON_HEADER,0xA4,0x95,0xBB,0x20,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Black" , GUI_GRAY,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x30,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Purple", GUI_PURPLE, {IBEACON_HEADER,0xA4,0x95,0xBB,0x40,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Orange", GUI_ORANGE, {IBEACON_HEADER,0xA4,0x95,0xBB,0x50,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Blue"  , GUI_BLUE,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x60,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Yellow", GUI_YELLOW, {IBEACON_HEADER,0xA4,0x95,0xBB,0x70,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
    {"Pink"  , GUI_PINK,   {IBEACON_HEADER,0xA4,0x95,0xBB,0x80,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE},0,0,0},
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
    TDM_CMD_GET_DATAPOINT,
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
        //printf("Tilt %s Temperature = %d Gravity =%f\n",tiltDB[i].colorName,tiltDB[i].data->temperature,tiltDB[i].data->gravity);
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

// This function returns a malloc'd copy of the front of the most recent datapoint ... this function should only be used 
// internally because it is not thread safe.
static tdm_tiltData_t *tdm_getDataPointCopy(tdm_tiltHandle_t handle)
{
    tdm_tiltData_t *dp;
    dp = malloc(sizeof(tdm_tiltData_t));
    memcpy(dp,(tdm_tiltData_t *)tiltDB[handle].data,sizeof(tdm_tiltData_t));

    dp->gravity = dp->gravity / tiltDB[handle].numDataPoints;
    dp->temperature = dp->temperature / tiltDB[handle].numDataPoints;

    return dp;
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
    tdm_tiltData_t *dp;

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

            case TDM_CMD_GET_DATAPOINT:
                dp = tdm_getDataPointCopy(msg.tilt);
                xQueueSend(msg.msg,&dp,0); // ARH 0 is probably a bad idea
            break;
        }
    }
}

char *tdm_getColorString(tdm_tiltHandle_t handle)
{
    return tiltDB[handle].colorName;
}


GUI_COLOR tdm_colorGUI(tdm_tiltHandle_t handle)
{
    return tiltDB[handle].color;
}

int tdm_getNumTilt()
{
    return NUM_TILT;
}

uint32_t tdm_getActiveTiltMask()
{
    uint32_t mask=0;
    for(int i=0;i<NUM_TILT;i++)
    {
        if(tiltDB[i].data)
            mask |= 1<<i;
    }
    return mask;
}

uint32_t tdm_getNumDataSeen(tdm_tiltHandle_t handle)
{
    return tiltDB[handle].numDataSeen;
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

tdm_tiltData_t *tdm_getTiltData(tdm_tiltHandle_t handle)
{
    QueueHandle_t respqueue;
    tdm_tiltData_t *rsp;

    if(handle<0 || handle>=NUM_TILT || tiltDB[handle].data == 0 )
        return 0;

    respqueue = xQueueCreate(1,sizeof(tdm_tiltData_t *));
    if(respqueue == 0)
        return 0;

    tdm_cmdMsg_t msg;
    msg.msg = respqueue;
    msg.tilt = handle;
    msg.cmd =  TDM_CMD_GET_DATAPOINT;
    if(xQueueSend(tdm_cmdQueue,&msg,0) != pdTRUE)
    {
        printf("failed to send to dmQueue\n");
    }
    xQueueReceive(respqueue,(void *)&rsp,portMAX_DELAY);
    vQueueDelete(respqueue);
    return rsp;
}