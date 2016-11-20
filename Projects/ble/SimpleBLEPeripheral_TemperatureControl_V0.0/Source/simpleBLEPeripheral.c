/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_sensor.h"
#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#include "PWMmode.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "SerialApp.h"

/* muliangxing add 20150103 begin*/
#include "Battservice.h"
/* muliangxing add 20150103 end*/
#include "Pairservice.h" /* muliangxing add 20150104 */

/*********************************************************************
 * MACROS
 */
#define MAX_TRAS_LENS               5

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20    //80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     20   //800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500  //80

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define JYUN_WAVE_MODE_MAX_LEVEL             6
#define JYUN_MIN_VIB_LEVEL                      20
#define JYUN_MIN_FREQ_LEVEL                     10
#define JYUN_APK_MODE_MAX_INDEX              15
#define JYUN_KEY_MODE_MAX_INDEX               9
#define JYUN_MODE_INCREASE_POWER_LEVEL       9

//TERIC ADD
#define SK_KEY_VALUE_POWER        0x01
#define SK_KEY_VALUE_PLUS          0x02
#define SK_KEY_VALUE_SMART        0x03
#define SK_KEY_VALUE_SUBSTRACT    0x04
#define SBP_PERIODIC_LONGPRESS_EVT            0x010
#define KFD_LEDS_START_TIMER_EVT            0x020
#define KFD_LEDS_STARTM_TIMER_EVT            0x040
#define SBP_PERIODIC_POWER_EVT            0x080
#define SBP_PERIODIC_ADC_EVT            0x100
#define KFD_SHUTDOWN_EVT                0x200
#define SBP_PERIODIC_WATCH_DOG_EVT    0x400  //WatchDog
#define KFD_LEDS_BLUE_TIMER_EVT       0x800
#define KFD_LEDS_DISCON_TIMER_EVT     0x1000

static uint8 g_curren_key_status = 0;//0: up;1:down;2:longpress
static uint8 g_curren_bt_connect_status = 0;//0: up;1:down;2:longpress
static uint8 g_bt_paire_status = 0;//0: up;1:connect;2:pair
static uint8 g_init_system_status = 0;
static uint8 g_need_report_level_status = 0;
static uint8 g_is_mode_screen_status = 0;
static uint8 g_is_mode_vib_status = 0;
static uint8 g_key_wave_mode = 0;
static uint8 g_key_shutdown = 0;
static uint8 g_charge_full_state = 0;


#define SN3730_DEV_ADDR               (0xC0 >> 1)
#define SN3730_HARDWARE_SWITCH_BIT     P0_6

#define ISKEYDOWN(curstatus, prestatus,keyid) ((curstatus&keyid)&&(!(prestatus&keyid))) 
#define ISKEYUP(curstatus, prestatus,keyid) ((!(curstatus&keyid))&&(prestatus&keyid))

#define ISKEYPRESSED(keyid,prestatus) (prestatus&keyid) 

/*********************************************************************
 * TYPEDEFS
 */


typedef enum 
{
   KEY_CODE_PLUS = 0,
   KEY_CODE_SMART,
   KEY_CODE_SUBSTRACT,
   KEY_CODE_POWER,
   KEY_COUNT
}KEY_CODE_VALUE;

typedef enum 
{
   KEY_EVENT_DOWN = 0,
   KEY_EVENT_UP,
   KEY_EVENT_LONGPRESS,
   KEY_EVENT_COUNT
}KEY_EVENT_TYPE;


/*********************************************************************
 * GLOBAL VARIABLES
 */
//static Act action[MAX_TRAS_LENS];
static uint8 currWave;
static uint8 WaveMode;
static uint8 advertising_enable;
static uint8 PowerOffMode;


static uint8 mCurrentlevel = 0;
static uint8 mledstate = 0;
static uint8 mledstatelow = 0;
static uint8 mIsllowbattery = 0;
static uint8 mIsprelowbattery = 0;
static uint8 mIschargingLed = 0;
static uint8 mIschargingFullLed = 0;
static uint8 mCountNumber = 0;
static uint8 is_long_change_status = 0;
static uint8 mPreKeys = 0x00;
static uint8 mKeyList[] = {HAL_KEY_SW_1,
						   HAL_KEY_SW_2,
						   HAL_KEY_SW_3,
						   HAL_KEY_SW_4};


#define CC2541_ADC_POWER_OFF_VALUE         398
#define CC2541_ADC_LOW_WARNING_VALUE       418
#define CC2541_ADC_VIB_POWER_OFF_VALUE     395
#define CC2541_ADC_VIB_LOW_WARNING_VALUE   408

/*********************************************************************
 * EXTERNAL VARIABLES
 */

static uint8 startflash[12][22] = {
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x10,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x10,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x10,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x10,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x8,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x4,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x4,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x2,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1,0x0,0x0,0x0,0x0,0x0},
{0x0,0x0,0x0,0x0,0x0,0x0,0x11,0x0,0x11,0x12,0x1B,0x0,0x11,0xC,0x15,0x0,0x11,0x0,0x0,0x0,0x0,0x0}
};

static uint8 shutdown[22] = {
  0x0,0x0,0x4,0x0,0x4,0x0,0xE,0x0,0x0,0x12,0x11,0x0,0x11,0x0,0x11,0x0,0x0,0x12,0xA,0x0,0x4,0xC
};		

static uint8 blue[22] = {
  0x0,0x0,0x4,0x4,0x6,0x10,0xC,0x2,0x6,0x8,0x4,0x4,0x4,0x4,0x6,0x8,0xC,0x2,0x6,0x10,0x4,0x4
};

static uint8 discon[22] = {
  0x0,0x0,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x4,0x0,0x0,0x4,0x4,0x0,0x0
};
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 key_permit_flag = 0;
static uint8 connect_status_flag = 0;
static uint8 blue_permit_flag = 0;
static uint8 mode = 0x1a;
static uint8 shutmode = 0x80;
static uint8 data = 0x08;
uint8 buf_0c[1]={0x00};	           /*定义0x0c寄存器缓冲区*/
 //static uint8 down_flag = 1;
uint8 buf_1[11];                   /*定义显示区1缓冲区*/
uint8 buf_2[11];                   /*定义显示区2缓冲区*/

uint8 buf_a = 0x3F;
uint8 buf_all = 0x3F;
uint8 buf_al = 0x3F;
static uint8 kmode = 0;


 uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  '6',
  '5',
  '0',
  'L', 
  'i',
  't',
  'e',
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "650Lite";
extern bStatus_t Pair_AddService( void );
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );
#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
static void sn3730_soft_shutdown();
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
static void jyun_init_watchdog(void)
{
  WDCTL = 0x00; //IDLE
  WDCTL &= ~(BV(0)|BV(1));
  WDCTL |= BV(3);
}

static void jyun_feed_watchdog(void)
{
  WDCTL = 0xA0 | (WDCTL & 0x0F);
  WDCTL = 0x50 | (WDCTL & 0x0F);
  //SerialPrintString("Feed dog\r\n");
}

static uint8 mIslowBattLevel = 0;
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  PowerOffMode = TRUE;

  SerialApp_Init(task_id);
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 1;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

  Batt_AddService( );     // Battery Service   /* muliangxing 20150103 */
  Pair_AddService( ); /* muliangxing add 20150103 end*/
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }


#if defined( CC2540_MINIDK )

 // SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );
#endif // #if defined( CC2540_MINIDK )

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )
  
  P0DIR |= BV(6);
  SN3730_HARDWARE_SWITCH_BIT = 0;
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  jyun_init_watchdog();
  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_WATCH_DOG_EVT, 500 );
  P0DIR &= ~BV(4);
  P0SEL &= ~BV(4);
  //osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, 200);
}


static void sn3730_soft_shutdown()
{
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    HalSensorWriteReg(0x00, &shutmode, 1);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_WATCH_DOG_EVT )
  {
    jyun_feed_watchdog();
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_WATCH_DOG_EVT, 500 );
    return ( events ^ SBP_PERIODIC_WATCH_DOG_EVT );
  }
  /*
  if ( events & SBP_PERIODIC_EVT )
  {
      if(g_init_system_status == 0){
              g_init_system_status = 1;
          if((SLEEPSTA & 0x18) == 0x10) //watchdog reset boot
          {
                 jyun_enter_shutdown_mode(0);
          }
          else
          {
            jyun_enter_shutdown_mode(1);
          }
      }else if(g_bt_paire_status == 1){
           PairSendStatus(1); 
           osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT,1000);
      }
      return ( events ^ SBP_PERIODIC_EVT );
  } */
  

  if ( events & KFD_LEDS_START_TIMER_EVT )
  {
    uint8 i, j;
    static uint8 a = 0x40;
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    if(kmode == 11)
    {
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_START_TIMER_EVT );
      sn3730_soft_shutdown();
      osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_STARTM_TIMER_EVT, 130 );
      return (events ^ KFD_LEDS_START_TIMER_EVT);
    }
    HalSensorWriteReg(0xFF, buf_0c, 1);
    HalSensorWriteReg(0x00, &mode, 1); 
    HalSensorWriteReg(0x0d, &data, 1);

    for(i=0x00, j=0;i<11;i++,j+=2)
    {
            buf_1[i]=startflash[kmode][j];
            buf_2[i]=startflash[kmode][j+1];      
    }
    
    HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x01,buf_1,11);
    HalSensorWriteReg(0x0e,buf_2,11);
    //HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x0c,buf_0c,0x01);
    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_START_TIMER_EVT, 130 );
    kmode++;

    // return unprocessed events
    return (events ^ KFD_LEDS_START_TIMER_EVT);
  }
  
   if ( events & KFD_LEDS_STARTM_TIMER_EVT )
  {
    uint8 i, j;
    static uint8 a = 0x00;
    static uint8 b = 1;
    if(a > 0x43)
    {
          a = 0;
          b = 1;
          kmode = 0;
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_STARTM_TIMER_EVT );
          sn3730_soft_shutdown();
          blue_permit_flag = 1;
          advertising_enable = TRUE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
          return (events ^ KFD_LEDS_STARTM_TIMER_EVT);      
    }
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    
    HalSensorWriteReg(0xFF, buf_0c, 1);
    HalSensorWriteReg(0x00, &mode, 1); // 问题出在这
    HalSensorWriteReg(0x0d, &data, 1);
    if(a == 0){
      for(i=0x00, j=0;i<11;i++,j+=2)
      {
              buf_1[i]=startflash[kmode][j];
              buf_2[i]=startflash[kmode][j+1];      
      }
    }
    a+=b;
    b++;
    HalSensorWriteReg(0x19, &a, 1);

    HalSensorWriteReg(0x01,buf_1,10); 
    HalSensorWriteReg(0x0e,buf_2,10);
    //HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x0c,buf_0c,0x01);
    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_STARTM_TIMER_EVT, 130 );    

    // return unprocessed events
    return (events ^ KFD_LEDS_STARTM_TIMER_EVT);
  }     
  
   if ( events & KFD_SHUTDOWN_EVT )
  {
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_START_TIMER_EVT );
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_STARTM_TIMER_EVT );    
    
    uint8 i, j;
    static uint8 a = 0x01;
    static uint8 down_flag = 0;
    static uint8 b = 2;
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    
    HalSensorWriteReg(0xFF, buf_0c, 1);
    HalSensorWriteReg(0x00, &mode, 1); // 问题出在这
    HalSensorWriteReg(0x0d, &data, 1);
    if(a == 1)
    {
      for(i=0x00, j=0;i<11;i++,j+=2)
      {
              buf_1[i]=shutdown[j];
              buf_2[i]=shutdown[j+1];      
      }
    }
    HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x01,buf_1,10); 
    HalSensorWriteReg(0x0e,buf_2,10);
    //HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x0c,buf_0c,0x01);
 
    if(down_flag == 1)
    {
        a-=b;
        b--;
        SerialPrintValue("dw_a = ", a, 10);
        SerialPrintString("\n");
        SerialPrintValue("dw_b = ", b, 10);
        SerialPrintString("\n");
        if(a < 0x01)
        {
          down_flag = 0;
          kmode = 0;
          a=1;b=2;
          PowerOffMode = TRUE;
          sn3730_soft_shutdown();
          SN3730_HARDWARE_SWITCH_BIT = 0;
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_START_TIMER_EVT );
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_STARTM_TIMER_EVT );
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_BLUE_TIMER_EVT );
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_DISCON_TIMER_EVT );
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_SHUTDOWN_EVT );
          if(advertising_enable == TRUE){
              advertising_enable = FALSE;
              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
          }
          if(g_curren_bt_connect_status == 1){
              GAPRole_TerminateConnection();
          }
          SerialPrintString("power off!");
          return (events ^ KFD_SHUTDOWN_EVT);
        }
        
    }
    else if(down_flag == 0)
    {
        a+=b;
        b++;
        SerialPrintValue("up_a = ", a, 10);
        SerialPrintString("\n");
        SerialPrintValue("up_b = ", b, 10);
        SerialPrintString("\n");
        if(a > 0x38)
        {
          down_flag = 1;
          b--;
        }
    }


    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_SHUTDOWN_EVT, 130 );    

    // return unprocessed events
    return (events ^ KFD_SHUTDOWN_EVT);
  }     
  
   if ( events & KFD_LEDS_BLUE_TIMER_EVT )
  {
    
    uint8 i, j;
    static uint8 a = 0x42;
    static uint8 down_flag = 0;
    static uint8 b = 11;
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    
    HalSensorWriteReg(0xFF, buf_0c, 1);
    HalSensorWriteReg(0x00, &mode, 1); // 问题出在这
    HalSensorWriteReg(0x0d, &data, 1);
    if(a == 0x42)
    {
      for(i=0x00, j=0;i<11;i++,j+=2)
      {
              buf_1[i]=blue[j];
              buf_2[i]=blue[j+1];      
      }
    }
    HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x01,buf_1,10); 
    HalSensorWriteReg(0x0e,buf_2,10);
    //HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x0c,buf_0c,0x01);
 
    a-=b;
    b--;
    SerialPrintValue("dw_a = ", a, 10);
    SerialPrintString("\n");
    SerialPrintValue("dw_b = ", b, 10);
    SerialPrintString("\n");
    if(a < 0x01)
    {
      down_flag = 0;
      kmode = 0;
      a=0x42;b=11;
      sn3730_soft_shutdown();
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_BLUE_TIMER_EVT ); 
      //SN3730_HARDWARE_SWITCH_BIT = 0;
      return (events ^ KFD_LEDS_BLUE_TIMER_EVT);
    }

    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_BLUE_TIMER_EVT, 130 );    

    // return unprocessed events
    return (events ^ KFD_LEDS_BLUE_TIMER_EVT);
  }
  
   if ( events & KFD_LEDS_DISCON_TIMER_EVT )
  {
    
    uint8 i, j;
    static uint8 a = 0x01;
    static uint8 down_flag = 0;
    static uint8 count = 0;
    static uint8 b = 2;
    HalSensorInit(SN3730_DEV_ADDR);      //设备初始化
    
    HalSensorWriteReg(0xFF, buf_0c, 1);
    HalSensorWriteReg(0x00, &mode, 1); // 问题出在这
    HalSensorWriteReg(0x0d, &data, 1);
    if(a == 0x01)
    {
      for(i=0x00, j=0;i<11;i++,j+=2)
      {
              buf_1[i]=discon[j];
              buf_2[i]=discon[j+1];      
      }
    }
    HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x01,buf_1,10); 
    HalSensorWriteReg(0x0e,buf_2,10);
    //HalSensorWriteReg(0x19, &a, 1);
    HalSensorWriteReg(0x0c,buf_0c,0x01);
 
    a+=b;
    b++;
    SerialPrintValue("dw_a = ", a, 10);
    SerialPrintString("\n");
    SerialPrintValue("dw_b = ", b, 10);
    SerialPrintString("\n");
    if(a > 0x42)
    {
      count++;
      a=0x1;b=2;
      if(count == 3)
      {
          count = 0;
          sn3730_soft_shutdown();
          osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_DISCON_TIMER_EVT ); 
          //SN3730_HARDWARE_SWITCH_BIT = 0;
          return (events ^ KFD_LEDS_DISCON_TIMER_EVT);
      }
    }

    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_DISCON_TIMER_EVT, 130 );    

    // return unprocessed events
    return (events ^ KFD_LEDS_DISCON_TIMER_EVT);
  }
  
  
#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER
//TERIC ADD
    if( events & SBP_PERIODIC_LONGPRESS_EVT)
    {
        g_curren_key_status = 1;
        if(PowerOffMode == TRUE){
                SerialPrintString("power up!");
                SerialPrintString("\n");
                PowerOffMode = FALSE;
                SN3730_HARDWARE_SWITCH_BIT = 1;
                osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_START_TIMER_EVT, 500 );     
        }else{
                SerialPrintString("power down!");
                SerialPrintString("\n");
                connect_status_flag = 0;
                blue_permit_flag = 0;
                osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_SHUTDOWN_EVT, 10 );
        }
      
        return ( events ^ SBP_PERIODIC_LONGPRESS_EVT);     
    }
//END

  if(events & SBP_PERIODIC_ADC_EVT){

	return (events ^ SBP_PERIODIC_ADC_EVT); 
  }  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
#pragma optimize=none

/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  SerialPrintString("handle key!");
  SerialPrintString("\n");
  //DispatchKeys(keys);
  if(!keys && PowerOffMode == TRUE){
         SerialPrintString("key up!");
         SerialPrintString("\n");
         SN3730_HARDWARE_SWITCH_BIT = 0;
  	 osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_LONGPRESS_EVT);
  }
  
  if(keys & HAL_KEY_SW_1){
         SerialPrintString("key down!");
         SerialPrintString("\n");
         osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_LONGPRESS_EVT, 2000);
  }
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      {
      }
      break;

    case GAPROLE_CONNECTED:
      {
          connect_status_flag = 1;
          if(blue_permit_flag == 1){
              osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_BLUE_TIMER_EVT, 50 );
          }
      }
      break;
    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
    case GAPROLE_WAITING:
      {
          if(connect_status_flag)
          {
            connect_status_flag = 0;
            osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_LEDS_DISCON_TIMER_EVT, 50 );
          }
         /* if(PowerOffMode == TRUE){
              advertising_enable = FALSE;
              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
          }*/
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
      }
      break;

    case GAPROLE_ERROR:
      {
      }
      break;

    default:
      {
      }
      break;

  }

}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  uint8 valueToCopy;
  uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
//  uint8 newValue;
  uint8 newStr[MAX_TRAS_LENS*4+1] = {0};

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:      
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newStr );
	  if(osal_strlen(newStr) >= 1)
	  {		
		switch(newStr[0])
		{
		  case 0x01:

		  	break;

		  case 0x02:
                    
		  	break;

		  case 0xFF:

		  	break;
		  default:
		  	break;
		}
		SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR1_LEN, &newStr);
	  }
      break;

    case SIMPLEPROFILE_CHAR3:
      break;

    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/
