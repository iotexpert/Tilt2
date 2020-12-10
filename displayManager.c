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

static void dm_nextScreen();

static bool dm_displayScreenSplashPre();
static void dm_displayScreenSplashInit();
static void dm_displayScreenSplashUpdate();
static bool dm_displayScreenSplashSeq();

static bool dm_displayScreenTablePre();
static void dm_displayScreenTableInit();
static void dm_displayScreenTableUpdate();
static bool dm_displayScreenTableSeq();

static bool dm_displaySinglePre();
static void dm_displaySingleInit();
static void dm_displaySingleUpdate();
static bool dm_displaySingleSeq();

typedef enum {
    SPLASH,
    TABLE,
    SINGLE,
} dm_screenName_t;

typedef enum {
    SCREEN_NEXT,  // Move to next screen
    SCREEN_AUTO,  // Toggle the automatic mode
    SCREEN_TABLE, // Change screen to table
} dm_cmd_t;

typedef struct {
    dm_cmd_t cmd;
} dm_cmdMsg_t;

static QueueHandle_t dm_cmdQueue;
static dm_screenName_t dm_currentScreen;

typedef struct {
    bool (*precheck)(void);   // return true if you can come to this screen
    void (*init)(void);       // draw the initial stuff
    void (*update)(void);     // update the data
    bool (*sequence)(void);   // sequence the data .. return true if you should go to the next screen
    dm_screenName_t next;
} dm_screenMgmt_t;

dm_screenMgmt_t screenList[] = {
    {dm_displayScreenSplashPre, dm_displayScreenSplashInit, dm_displayScreenSplashUpdate, dm_displayScreenSplashSeq, TABLE},
    {dm_displayScreenTablePre , dm_displayScreenTableInit, dm_displayScreenTableUpdate, dm_displayScreenTableSeq, SINGLE},
    {dm_displaySinglePre, dm_displaySingleInit, dm_displaySingleUpdate, dm_displaySingleSeq, TABLE},
};

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
    dm_cmdMsg_t msg;
    bool autoRotate=true;
    dm_cmdQueue = xQueueCreate(10,sizeof(dm_cmdMsg_t));

    /* Initialize the display controller */
    mtb_st7789v_init8(&tft_pins);
    GUI_Init();

    dm_currentScreen = SPLASH;

    dm_displayScreenSplashInit();

    for(;;)
    {
        if(xQueueReceive(dm_cmdQueue,&msg,5000) == pdPASS) // Got a command
        {
            switch(msg.cmd)
            {
                case SCREEN_NEXT:
                dm_nextScreen();
                break;
                case SCREEN_AUTO:
                autoRotate = ! autoRotate;
                printf("AutoRotate =%s\n",autoRotate?"True":"False");
                break;
                case SCREEN_TABLE:
                dm_currentScreen = TABLE;
                (*screenList[dm_currentScreen].init)();
                (*screenList[dm_currentScreen].update)();
                break;

            }

        }
        else // otherwise just update the screen
        {
            if(autoRotate)
                dm_nextScreen();
            else
                (*screenList[dm_currentScreen].update)();
        }
    }
}

static void dm_nextScreen()
{
    if((*screenList[dm_currentScreen].sequence)())
    {
        if((*screenList[screenList[dm_currentScreen].next].precheck)())
        {
            dm_currentScreen = screenList[dm_currentScreen].next;
            (*screenList[dm_currentScreen].init)();
        }
    }
    (*screenList[dm_currentScreen].update)();
}




/////////////// Views ////////////////////

#define TOP_MARGIN (4)
#define LINE_MARGIN (2)
#define ROW_Y(row) (TOP_MARGIN + (row)*(LINE_MARGIN+GUI_GetFontSizeY()))

#define CENTER_X (160)
#define CENTER_Y (120)



////////////////////////////////////////////////////////////////////////////////
//
// Splash
// 
////////////////////////////////////////////////////////////////////////////////
static bool dm_displayScreenSplashPre()
{
    return true;
}

static void dm_displayScreenSplashInit()
{
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_DrawBitmap(&bmIOTexpert_Logo_Vertical320x240,0,17);

}
static void dm_displayScreenSplashUpdate()
{
}

static bool dm_displayScreenSplashSeq()
{
    return true;
}


////////////////////////////////////////////////////////////////////////////////
//
// Table
// 
////////////////////////////////////////////////////////////////////////////////

#define TABLE_FONT (GUI_FONT_24B_ASCII)
#define TABLE_BGCOLOR (GUI_BLACK)
#define TABLE_HEAD_BGCOLOR (GUI_WHITE)

#define TABLE_NAME_LEFT_X (0)
#define TABLE_GRAV_LEFT_X (120)
#define TABLE_TEMP_LEFT_X (220)

#define TABLE_NAME_RIGHT_X (119)
#define TABLE_GRAV_RIGHT_X (219)
#define TABLE_TEMP_RIGHT_X (319)

#define TABLE_NAME_CENTER_X (TABLE_NAME_LEFT_X+(TABLE_NAME_RIGHT_X-TABLE_NAME_LEFT_X)/2)
#define TABLE_GRAV_CENTER_X (TABLE_GRAV_LEFT_X+(TABLE_GRAV_RIGHT_X-TABLE_GRAV_LEFT_X)/2)
#define TABLE_TEMP_CENTER_X (TABLE_TEMP_LEFT_X+(TABLE_TEMP_RIGHT_X-TABLE_TEMP_LEFT_X)/2)

static bool dm_displayScreenTablePre()
{
    return true;
}

static void dm_displayScreenTableInit()
{
    GUI_SetColor(TABLE_HEAD_BGCOLOR);
    GUI_SetBkColor(TABLE_BGCOLOR);
    GUI_SetFont(TABLE_FONT);
    GUI_Clear();

    GUI_FillRect(0,0,320,GUI_GetFontSizeY()+ TOP_MARGIN);

    GUI_SetTextMode(GUI_TM_REV);
    GUI_SetTextAlign(GUI_TA_CENTER);
    GUI_DispStringHCenterAt("Name",TABLE_NAME_CENTER_X,TOP_MARGIN);
    GUI_DispStringHCenterAt("Gravity",TABLE_GRAV_CENTER_X,TOP_MARGIN);
    GUI_DispStringHCenterAt("Temp",TABLE_TEMP_CENTER_X,TOP_MARGIN);

    GUI_DrawLine(TABLE_NAME_RIGHT_X,0,TABLE_NAME_RIGHT_X,240);
    GUI_DrawLine(TABLE_GRAV_RIGHT_X,0,TABLE_GRAV_RIGHT_X,240);
}

static void dm_displayScreenTableUpdate()
{
    uint32_t activeTilts =tdm_getActiveTiltMask();

    char buff[64];
    
    GUI_SetColor(TABLE_HEAD_BGCOLOR);
    GUI_SetBkColor(TABLE_BGCOLOR);
    
    GUI_SetFont(TABLE_FONT);
    GUI_SetTextMode(GUI_TEXTMODE_NORMAL);
    GUI_SetTextAlign(GUI_TA_CENTER);

    int row;
    for(int i=0;i<tdm_getNumTilt();i++)
    {
        row = i+1;

        GUI_SetColor(tdm_colorGUI(i));
        GUI_DispStringHCenterAt(tdm_getColorString(i), TABLE_NAME_CENTER_X, ROW_Y(row));

        if(1<<i & activeTilts)
        {

            tdm_tiltData_t *response = tdm_getTiltData(i);

            sprintf(buff,"%1.3f",response->gravity);
            GUI_DispStringHCenterAt(buff, TABLE_GRAV_CENTER_X, ROW_Y(row));
            sprintf(buff,"%02d",response->temperature);
            GUI_DispStringHCenterAt(buff, TABLE_TEMP_CENTER_X, ROW_Y(row));
                        
            free(response);
        }
        else
        {
            GUI_DispStringHCenterAt("-----", TABLE_GRAV_CENTER_X, ROW_Y(row));
            GUI_DispStringHCenterAt("--", TABLE_TEMP_CENTER_X, ROW_Y(row));

        }
    }  
}

static bool dm_displayScreenTableSeq()
{
    return true;
}


////////////////////////////////////////////////////////////////////////////////
//
// Single
// 
////////////////////////////////////////////////////////////////////////////////

#define SINGLE_FONT (GUI_FONT_32B_ASCII)
#define SINGLE_BGCOLOR (GUI_BLACK)
#define SINGLE_LABEL_X (160)
#define SINGLE_VALUE_X (SINGLE_LABEL_X + 10)

#define SINGLE_GRAV_ROW (2)
#define SINGLE_TEMP_ROW (3)
#define SINGLE_TXPOWER_ROW (4)
#define SINGLE_RSSI_ROW (5)
#define SINGLE_TIME_ROW (6)


static tdm_tiltHandle_t currentSingle = 0xFF;

static bool dm_displaySinglePre()
{
    uint32_t activeTilts =tdm_getActiveTiltMask();
    if(activeTilts == 0)
        return false;

    for(int i=0;i<tdm_getNumTilt();i++)
    {
        if(1<<i & activeTilts)
        {
            currentSingle = i;
            break;
        }
    }

    return true;
}

static void dm_displaySingleInit()
{
    GUI_SetBkColor(GUI_BLACK);
    GUI_SetFont(GUI_FONT_32B_ASCII);
    GUI_Clear();

    GUI_SetColor(tdm_colorGUI(currentSingle));

    GUI_FillRect(0,0,320,ROW_Y(1));
    
    GUI_SetTextAlign(GUI_TA_LEFT);
    GUI_SetTextMode(GUI_TM_REV);
    GUI_DispStringHCenterAt(tdm_getColorString(currentSingle), CENTER_X,ROW_Y(0) );

    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);

    GUI_DispStringAt("Gravity: ", SINGLE_LABEL_X,ROW_Y(SINGLE_GRAV_ROW) );
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispStringAt("Temp: ",    SINGLE_LABEL_X,ROW_Y(SINGLE_TEMP_ROW) );
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispStringAt("TxPower: ", SINGLE_LABEL_X,ROW_Y(SINGLE_TXPOWER_ROW) );
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispStringAt("RSSI: ",    SINGLE_LABEL_X,ROW_Y(SINGLE_RSSI_ROW) );
    GUI_SetTextAlign(GUI_TA_RIGHT | GUI_TA_VCENTER);
    GUI_DispStringAt("Time: ",    SINGLE_LABEL_X,ROW_Y(SINGLE_TIME_ROW) );

}

static void dm_displaySingleUpdate()
{

    uint32_t activeTilts =tdm_getActiveTiltMask();

    char gravString[10];
    char tempString[10];
    char txPowerString[10];
    char rssiString[10];
    char timeString[64];
       
    GUI_SetBkColor(GUI_BLACK);
    GUI_SetFont(GUI_FONT_32B_ASCII);
    GUI_SetColor(tdm_colorGUI(currentSingle));
    
    if(1<<currentSingle & activeTilts)
    {
        tdm_tiltData_t *response = tdm_getTiltData(currentSingle);
        sprintf(gravString,"%1.3f",response->gravity);
        sprintf(tempString,"%02d",response->temperature);
        sprintf(txPowerString,"%d",response->txPower);
        sprintf(rssiString,"%2d",response->rssi);

        int seconds = (xTaskGetTickCount()/1000- response->time);
        int days = seconds/(24*60*60);
        seconds = seconds - days*(24*60*60);
        int hours = seconds/(60*60);
        seconds = seconds - hours*(60*60);
        int minutes = seconds/60;
        seconds = seconds - (minutes * 60);
        
        sprintf(timeString,"%02d:%02d:%02d:%02d",days,hours,minutes,seconds);                       
        free(response);
    }

    else
    {
        sprintf(gravString,"-----");
        sprintf(tempString,"--");
        sprintf(txPowerString,"---");
        sprintf(rssiString,"--");
        sprintf(timeString,"---");                       
    }

    GUI_DispStringAtCEOL(gravString, SINGLE_VALUE_X,ROW_Y(SINGLE_GRAV_ROW) );
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispStringAtCEOL(tempString,    SINGLE_VALUE_X,ROW_Y(SINGLE_TEMP_ROW) );
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispStringAtCEOL(txPowerString, SINGLE_VALUE_X,ROW_Y(SINGLE_TXPOWER_ROW) );
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispStringAtCEOL(rssiString,    SINGLE_VALUE_X,ROW_Y(SINGLE_RSSI_ROW) );
    GUI_SetTextAlign(GUI_TA_LEFT | GUI_TA_VCENTER);
    GUI_DispStringAtCEOL(timeString,    SINGLE_VALUE_X,ROW_Y(SINGLE_TIME_ROW) );
}

static bool dm_displaySingleSeq()
{
    uint32_t activeTilts =tdm_getActiveTiltMask();
    if(activeTilts == 0)
        return true;

    for(int i=currentSingle+1;i<tdm_getNumTilt();i++)
    {
        if(1<<i & activeTilts)
        {
            currentSingle = i;
            dm_displaySingleInit();
            return false;
        }
    }
    return true;
}



////////////////////////////////////////////////////////////////////////////////
//
// commands
//
////////////////////////////////////////////////////////////////////////////////

void dm_submitNextScreenCmd()
{
    dm_cmdMsg_t msg;
    msg.cmd = SCREEN_NEXT;
    xQueueSend(dm_cmdQueue,&msg,0);

}

void dm_submitAutoCmd()
{
    dm_cmdMsg_t msg;
    msg.cmd = SCREEN_AUTO;
    xQueueSend(dm_cmdQueue,&msg,0);

}

void dm_submitTable()
{
    dm_cmdMsg_t msg;
    msg.cmd = SCREEN_TABLE;
    xQueueSend(dm_cmdQueue,&msg,0);

}