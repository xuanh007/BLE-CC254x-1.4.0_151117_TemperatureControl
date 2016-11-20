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
#include "iic.h"
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
#include "string.h"

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
#define DEFAULT_ADVERTISING_INTERVAL          800

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


//TERIC ADD
#define SK_KEY_VALUE_POWER        0x01
#define SK_KEY_VALUE_PLUS          0x02
#define SK_KEY_VALUE_SMART        0x03
#define SK_KEY_VALUE_SUBSTRACT    0x04

/* 目前空余4个timer：0x0002 0x0004 0x0040 0x10000 */
#define SBP_PERIODIC_B1_3_STORE_EVT            0x010
//#define SBP_PERIODIC_EVT                0x080
#define SBP_PERIODIC_IIC_WRITE_DOWN_EVT            0x100
#define SBP_PERIODIC_WATCH_DOG_EVT    0x400  //WatchDog

#define SBP_PERIODIC_CHECK_EVT            0x020
#define SBP_PERIODIC_READ_TO_NOTIYF_EVT         0x200
#define SBP_PERIODIC_START_DATA_STORE_EVT       0x800
#define SBP_PERIODIC_IIC_WRITE_PERMIT_EVT   0x2000
#define SBP_PERIODIC_CONN_READ_EVT  0x4000 
//#define SBP_PERIODIC_B1_3_WRITE_EVT  0x8000

#define SN3730_DEV_ADDR               (0xC0 >> 1)

#define ISKEYDOWN(curstatus, prestatus,keyid) ((curstatus&keyid)&&(!(prestatus&keyid))) 
#define ISKEYUP(curstatus, prestatus,keyid) ((!(curstatus&keyid))&&(prestatus&keyid))

#define ISKEYPRESSED(keyid,prestatus) (prestatus&keyid) 

#define CC2541_ADC_POWER_OFF_VALUE         402
#define CC2541_ADC_LOW_WARNING_VALUE       952
#define CC2541_ADC_LED_POWER_OFF_VALUE     395
#define CC2541_ADC_LED_LOW_WARNING_VALUE   408
#define CC2541_ADC_CHARGING_FULLED_VALUE   482


/* charg interrupt is at P1.6 */
#define HAL_KEY_SW_6_PORT   P1
#define HAL_KEY_SW_6_BIT    BV(6)
#define HAL_KEY_SW_6_SEL    P1SEL
#define HAL_KEY_SW_6_DIR    P1DIR

/* SW_4 is at P1.7 */
#define HAL_KEY_SW_7_PORT   P1
#define HAL_KEY_SW_7_BIT    BV(7)
#define HAL_KEY_SW_7_SEL    P1SEL
#define HAL_KEY_SW_7_DIR    P1DIR

#define SLAVE_DEV_ADDR    (0xA0 >> 1)

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
//static Act action[MAX_TRAS_LENS];

static uint8 advertising_enable;
static uint8 g_start_data = 0;
static uint8 pre_lens = 0;
static uint8 ready_to_notify = 0;

static uint8 g_send_buff[200];
static uint8 g_rsv_buff[220];
static uint8 g_read_to_notify[20];

static uint8 g_con_read_flag = 0;
static uint8 g_set_write_flag = 0;

static uint16 g_datas_count = 0;
static uchar B1 = 0;
static uchar B2 = 0;
static uchar B3 = 0;
static uint32 addr = 0x0100;

uint8 flag_power_on = 1;//进低功耗

struct read_data read_y;

static uint8 g_total_datas = 0;
static uint8 g_rsv_total_datas = 0;
static uint8 g_check_evt = 0;
static uint8 g_is_busy = 0;

static uint8 g_is_full = 0;
static uint8 times = 0;
static uint8 y_times = 0;


typedef enum{
    NO_EVT_RUNNING = 0,
    CONN_READ_EVT,
    CHECK_EVT,
    IIC_WRITE_PERMIT_EVT,
    START_DATA_STORE_EVT,
    READ_TO_NOTIYF_EVT,
    EVT_TIMES
}EVTS_LOCK;

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/* SN3730寄存器参数设置 */
uint8 buf_0c[1]={0x00};	           /*定义0x0c寄存器缓冲区*/
uint8 buf_1[11];                   /*定义显示区1缓冲区*/
uint8 buf_2[11];                   /*定义显示区2缓冲区*/

uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x9,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'T',
  'e',
  'p',
  'H', 
  'e',
  'a',
  't',
  'B',
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
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "TepHeatB";
extern bStatus_t Pair_AddService( void );
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



void power_saving_in(void)
{
     pwrmgr_attribute.pwrmgr_device = PWRMGR_BATTERY;   //Vivian
     flag_power_on = 1;
}
void power_saving_out(void)
{
    pwrmgr_attribute.pwrmgr_device = PWRMGR_ALWAYS_ON;   //Vivian
    flag_power_on = 0;
}



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
  SerialApp_Init(task_id);
  SerialPrintString("start!");
  
  Disable_IIC_Ports(); //disable IIC
   
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      advertising_enable = TRUE;
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
  
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  jyun_init_watchdog();
  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_WATCH_DOG_EVT, 500 );
  //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_TIME_DATA_STORE_EVT, 5*60*1000 );
  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, (uint32)10*60*1000 );
  P0DIR &= ~BV(6);   //定义P0.6为输入口
  P0SEL &= ~BV(6);   //定位P0.6为一般GPIO
  P0DIR |= BV(7);    //定义P0.7为输出口
  P0SEL &= ~BV(7);   //定位P0.6为一般GPIO
  //P0_6 = 0;
  P0_7 = 0;
  //P0INP |= BV(4);
  osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, 200);
}


static void byun_enter_shutdown_mode(uint8 restart)
{
  if(restart == 1){  //硬件重启
          P0DIR &= ~BV(6);   //定义P0.6为输入口
          P0SEL &= ~BV(6);   //定位P0.6为一般GPIO
          P0DIR |= BV(7);    //定义P0.7为输出口
          P0SEL &= ~BV(7);   //定位P0.6为一般GPIO
          //P0_6 = 0;
          P0_7 = 0;
          g_check_evt=0;
          power_saving_in();
          SerialPrintString("nomalreboot!");
          Disable_IIC_Ports();
          advertising_enable = TRUE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
  }else if(restart == 0){  //看门狗重启
          P0DIR &= ~BV(6);   //定义P0.6为输入口
          P0SEL &= ~BV(6);   //定位P0.6为一般GPIO
          P0DIR |= BV(7);    //定义P0.7为输出口
          P0SEL &= ~BV(7);   //定位P0.6为一般GPIO
          //P0_6 = 0;
          P0_7 = 0;
          g_check_evt=0;
          power_saving_in();
          SerialPrintString("dogreboot!");
          Disable_IIC_Ports();
          advertising_enable = TRUE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
  }
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

    //SerialPrintValue("control_status = ", control_status, 10);
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 1000 );

     // HalSensorInit(SLAVE_DEV_ADDR);

    //  HalSensorWriteReg(0x30, &c, 6);
     // HalSensorWriteReg(0x30, &c, 6);
      /*HalSensorWriteReg(0x30, b, 168);
      HalSensorWriteReg(0xE0, &a[174], 6);
      HalSensorWriteReg(0xF0, &a[180], 4);*/
    
    
   }

  if ( events & SBP_PERIODIC_WATCH_DOG_EVT )
  {
    jyun_feed_watchdog();
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_WATCH_DOG_EVT, 500 );
    return ( events ^ SBP_PERIODIC_WATCH_DOG_EVT );
  }
  
  if ( events & SBP_PERIODIC_IIC_WRITE_DOWN_EVT )
  {
    P0_7 = 0;
    g_is_busy = NO_EVT_RUNNING;
    Disable_IIC_Ports();
    power_saving_in();
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_DOWN_EVT);
    osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_STORE_EVT );
    return ( events ^ SBP_PERIODIC_IIC_WRITE_DOWN_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
      SerialPrintString("events in\t\n!");

      SerialPrintString("reboot!");
      if((SLEEPSTA & 0x18) == 0x10) //watchdog reset boot
      {
            byun_enter_shutdown_mode(0);
      }
      else
      {
            byun_enter_shutdown_mode(1);  //硬件重启
      }

      return ( events ^ SBP_PERIODIC_EVT );
  } 
  
  if ( events & SBP_PERIODIC_IIC_WRITE_PERMIT_EVT )
  {
      static uint8 a = 0;
      static uint8 b = 0;
      static uint8 c = 0;
      SerialPrintString("SBP_PERIODIC_IIC_WRITE_PERMIT_EVT!!!!\t\n!");
      if((P0_6 == 0) && ((g_is_busy == NO_EVT_RUNNING) || (g_is_busy == IIC_WRITE_PERMIT_EVT)))
      {
          P0_7 = 1;
          g_is_busy = IIC_WRITE_PERMIT_EVT;
          SerialPrintString("P0_6 = 0!!!!\t\n!");
          if(g_set_write_flag < 6)
          {
                   //SerialPrintString("aaaaaaaaaaaaaaa!!!!\t\n!");
                   Write_Add(0xa0,0x00,0x10+g_set_write_flag,g_rsv_buff[g_set_write_flag]);//向器件写一个数据: 
                   g_set_write_flag++;
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 5 );
          }else if(g_set_write_flag > 5 && g_set_write_flag < 90){
                   Write_Add(0xa0,0x00,0x30+a,g_rsv_buff[g_set_write_flag]);//向器件写一个数据: 
                   a++;
                   g_set_write_flag++;
                   //SerialPrintString("bbbbbbbbbbbbbbbb!!!!\t\n!");
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 5 );
          }else if(g_set_write_flag > 89 && g_set_write_flag < 96){
                   Write_Add(0xa0,0x00,0x90+b,g_rsv_buff[g_set_write_flag]);//向器件写一个数据: 
                   b++;
                   g_set_write_flag++;
                   //SerialPrintString("ccccccccccccccc!!!!\t\n!");
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 5 );
          }else if(g_set_write_flag > 95 && g_set_write_flag < 101){
                   Write_Add(0xa0,0x00,0xA0+c,g_rsv_buff[g_set_write_flag]);//向器件写一个数据: 
                   c++;
                   g_set_write_flag++;
                   //SerialPrintString("dddddddddddddddd!!!!\t\n!");
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 5 );
          }else{
                   g_set_write_flag = 0;
                   a = 0;
                   b = 0;
                   c = 0;
                   Write_Add(0xa0,0x00,0x18,0x55);
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_DOWN_EVT, 5 );
                   SerialPrintString("eeeeeeeeeeeeeee!!!!\t\n!");
                   osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT);
          }
      }
      else{
          SerialPrintString("P0_6 = 1!!!!\t\n!");
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 800 );
      }
      return ( events ^ SBP_PERIODIC_IIC_WRITE_PERMIT_EVT );
  } 
  
  if ( events & SBP_PERIODIC_READ_TO_NOTIYF_EVT )
  {
     // SerialPrintString("SBP_PERIODIC_READ_TO_NOTIYF_EVT!!!!");
          attHandleValueNoti_t noti;
        
          //将写入数组的20个数据发给APP
          uint8 beg_start[2] = {0x56,0x78};
          uint8 beg_end[2] = {0x9A,0xBC};
          uint8 addrl = 0;
          uint8 addrh = 0;
          uint8 i;
          static uint8 date_flag = 0;
          static uint8 date_lens = 0;
          static uint8 end_flag = 0;
          static uint8 count_flag = 0;  //用来记录发了多少个包
          uint32 startaddr = 0x0100;
          uint16 addrdec;
          addrdec = addr-startaddr;
          addrl = (addr & 0xff);
          addrh = ((addr >> 8) & 0xff);
          //SerialPrintValue("addrl = ", addrl, 16);
          //SerialPrintValue("addrh = ", addrh, 16);
          /*
          uint8 j;
          uint8 bags = g_total_datas/20;
          noti.handle = 0x2E;
          noti.len = 20;
          SerialPrintValue("bags = ", bags, 10);
          SerialPrintString("\t\n!");

          for(i=0; i<bags; i++){
                for(j=0; j<20; j++){
                    noti.value[j] = g_send_buff[i*20+j];
                    //lenth -= 20;
                }
                GATT_Notification(0, &noti, FALSE);
          }
                  P0_7 = 0;
                  g_is_busy = NO_EVT_RUNNING;      */    
             
            
          if(count_flag < times){
              date_lens = 20;
              end_flag = 0;
              SerialPrintValue("count_flag = ", count_flag, 10);
              SerialPrintString("\t\n!");
          }else{
              date_lens = y_times;
              end_flag = 1;
              SerialPrintValue("xxxxxy_times = ", y_times, 10);
              SerialPrintString("\t\n!");
              
          }
          //38f409
          //??????重新启动一个事件来读取发送数据
          if(date_flag < date_lens){
              g_read_to_notify[date_flag] = Read_Add(0xa0,addrh,addrl,0xa1);
              date_flag++;
              addr++;
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_READ_TO_NOTIYF_EVT, 5 );
          }else{
              date_flag = 0;
              SerialPrintValue("count_flag++++++ = ", count_flag, 10);
              SerialPrintString("\t\n!");
              if(count_flag == 0){
              
                  //2、通过特征值4，notify至APP
                  noti.handle = 0x2E;
                  noti.len = 2;
                  noti.value[0] = beg_start[0];
                  noti.value[1] = beg_start[1];
                  GATT_Notification(0, &noti, FALSE);
              }
              noti.len = date_lens;
              noti.handle = 0x2E;
              for(i=0; i<date_lens; i++){
                  noti.value[i] = g_read_to_notify[i];
                  //lenth -= 20;
              }
              //noti.value = g_read_to_notify;
              GATT_Notification(0, &noti, FALSE);
              SerialPrintValue("end_flag = ", end_flag, 10);
              SerialPrintString("\t\n!");
              if(end_flag == 1){
                  SerialPrintString("end of notify");
                  SerialPrintString("\t\n!");
                  noti.handle = 0x2E;
                  noti.len = 2;
                  noti.value[0] = beg_end[0];
                  noti.value[1] = beg_end[1];
                  GATT_Notification(0, &noti, FALSE);
                  //end   到此数据传输全部完成
                  count_flag = 0;
                  date_lens = 0;
                  Disable_IIC_Ports();
                  P0_7 = 0;
                  g_is_busy = NO_EVT_RUNNING;
                  osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT );
                  osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_READ_TO_NOTIYF_EVT );
                  power_saving_in();
              }else{
                  count_flag++;
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_READ_TO_NOTIYF_EVT, 5 );
              }
          }
      

      return ( events ^ SBP_PERIODIC_READ_TO_NOTIYF_EVT );
  } 

  if ( events & SBP_PERIODIC_CHECK_EVT )
  {
    attHandleValueNoti_t noti;
    if((ready_to_notify == 1) && (g_check_evt == 1)){
        SerialPrintString("data send!!!!\t\n!");
        uint8 i,j;
        //attHandleValueNoti_t noti;
        //1、读取IIC器件数据
        uint8 beg_start[2] = {0x41,0x52};
        uint8 beg_end[2] = {0x63,0x74};
        
        //2、通过特征值4，notify至APP
        noti.handle = 0x2E;
        noti.len = 2;
        noti.value[0] = beg_start[0];
        noti.value[1] = beg_start[1];
        GATT_Notification(0, &noti, FALSE);
        
        uint8 bags = g_total_datas/20;
        noti.len = 20;
        SerialPrintValue("bags = ", bags, 10);
        SerialPrintString("\t\n!");
        for(i=0; i<bags; i++){
              for(j=0; j<20; j++){
                  noti.value[j] = g_send_buff[i*20+j];
                  //lenth -= 20;
              }
              GATT_Notification(0, &noti, FALSE);
        }
        
        noti.len = g_total_datas%20;
        SerialPrintValue("noti.len = ", noti.len, 10);
        SerialPrintString("\t\n!");
        if(noti.len != 0){
          for(j=0; j<noti.len; j++){
              noti.value[j] = g_send_buff[bags*20+j];
              //lenth -= 20;
          }
          GATT_Notification(0, &noti, FALSE);    
        }
      
        noti.len = 2;
        noti.value[0] = beg_end[0];
        noti.value[1] = beg_end[1];
        GATT_Notification(0, &noti, FALSE);
        // Set timer for first periodic event
        //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 1000 );
        
        ready_to_notify = 2;
        g_is_busy = NO_EVT_RUNNING;
    }else if((ready_to_notify == 2) && (g_check_evt == 2)){  //传输图标温度数据
      
        SerialPrintString("tttttttttttttttttttttttttttttttttttttt\t\n!");
        if((P0_6 == 0) && ((g_is_busy == NO_EVT_RUNNING) || (g_is_busy == CHECK_EVT))){
            P0_7 = 1;
            g_is_busy = CHECK_EVT;
            //建一个长度为20的数组，根据已经保存的数据长度：g_datas_count，分别将已经保存的数据发给手机APP
            if(g_datas_count == 0){
                noti.handle = 0x2E;
                noti.len = 2;
                noti.value[0] = 0xEE;
                noti.value[1] = 0xFF;
                GATT_Notification(0, &noti, FALSE);
            }else{

                if(g_is_full == 1)
                {
                    times = (1008*3)/20;
                    y_times = (1008*3)%20;
                }else{
                    times = (g_datas_count*3)/20;
                    y_times = (g_datas_count*3)%20;
                }
                SerialPrintValue("mmtimes = ", times, 10);
                SerialPrintString("\t\n!");
                SerialPrintValue("mmy_times = ", y_times, 10);
                SerialPrintString("\t\n!");
                //for(i=0; i<times+1; i++){
                  
                //1、启动定时器事件，读取已经存储的数据。
                //一旦断开，丢失上次数据，从第一个地址开始存储
                osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_READ_TO_NOTIYF_EVT, 5 );
                osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT);
              
                //2、发送
                    
                //}
            }
        }else{
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 800 );
        }
    }else{
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 800 );
        }
    
    return ( events ^ SBP_PERIODIC_CHECK_EVT );
  }
  
  
  
  /*if ( events & SBP_PERIODIC_B1_3_WRITE_EVT )
  {
      SerialPrintString("store B1/B2/B2 datas!\t\n!");


      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_WRITE_EVT, 5 );
      return ( events ^ SBP_PERIODIC_B1_3_WRITE_EVT );
  } */
  
   
  if ( events & SBP_PERIODIC_B1_3_STORE_EVT )
  {
      //SerialPrintString("store B1/B2/B2 datas!\t\n!");
      
          uint8 addrl = 0;
          uint8 addrh = 0;
          static uint8 delay_flag = 0;
          uint32 startaddr = 0x0100;
          uint16 addrdec;
          addrdec = addr-startaddr;
          addrl = (addr & 0xff);
          addrh = ((addr >> 8) & 0xff);
          SerialPrintValue("addrl = ", addrl, 16);
          SerialPrintValue("addrh = ", addrh, 16);
          if(delay_flag == 0){
              if((addrdec%3) == 0){
                  Write_Add(0xa0,addrh,addrl,B1);//向器件写一个数据: 
                  addr++;
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_STORE_EVT, 5 );
              }else if(addrdec%3 == 1){
                  Write_Add(0xa0,addrh,addrl,B2);//向器件写一个数据: 
                  addr++;
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_STORE_EVT, 5 );
              }else if(addrdec%3 == 2){
                  Write_Add(0xa0,addrh,addrl,B3);//向器件写一个数据: 
                  addr++;
                  g_datas_count++;
                  delay_flag = 1;
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_STORE_EVT, 5 );
              }
          }else if(delay_flag){
                  if(g_datas_count == 1008){
                      g_is_full = 1;
                      g_datas_count = 0;
                      addr = 0x0100;
                  }
                  delay_flag = 0;
                  Write_Add(0xa0,0x00,0x18,0x55);
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_DOWN_EVT, 5 );
          }
          
          SerialPrintString("\t\n!");
     
      //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_TIME_DATA_STORE_EVT, 10*60*1000 );
      return ( events ^ SBP_PERIODIC_B1_3_STORE_EVT );
  }  
  
  
  
  if ( events & SBP_PERIODIC_CONN_READ_EVT )
  {
      static uint8 aa = 0;
      static uint8 bb = 0;
      static uint8 cc = 0;
      uchar year=0;
      //static uchar cur_tmp = 0;
     // SerialPrintString("read & notify B1/B2/B2 datas!\t\n!");
      HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
      if((P0_6 == 0) && ((g_is_busy == NO_EVT_RUNNING) || (g_is_busy == CONN_READ_EVT)))
      {
          P0_7 = 1;
          g_is_busy = CONN_READ_EVT;
          if(g_con_read_flag < 6)
          {
                   g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x10+g_con_read_flag,0xa1);
                   g_con_read_flag++;
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 5 );
          }else if(g_con_read_flag > 5 && g_con_read_flag < 90){
                   //SerialPrintString("1111111111111111111111!\t\n!");
                   g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x30+aa,0xa1);
                   aa++;
                   g_con_read_flag++;
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 5 );
          }else if(g_con_read_flag > 89 && g_con_read_flag < 96){
                   //SerialPrintString("22222222222222222!\t\n!");
                   g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x90+bb,0xa1);
                   bb++;
                   g_con_read_flag++;
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 5 );
          }else if(g_con_read_flag > 95 && g_con_read_flag < 101){
                   //SerialPrintString("3333333333333333333333!\t\n!");
                   g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0xA0+cc,0xa1);
                   cc++;
                   g_con_read_flag++;
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 5 );
          }else if(g_con_read_flag == 101){
                    
                    year = Read_Add(0xa0,0x00,0xE0,0xa1);
                    if((year == 16) || (year == 20) || (year == 24)){
                          g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x20,0xa1);
                          SerialPrintValue("16_B1 = ", B1, 10);
                          SerialPrintString("\t\n!");
                    }else if((year == 17) || (year == 21) || (year == 25)){
                          g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x22,0xa1);
                          SerialPrintValue("16_B1 = ", B1, 10);
                          SerialPrintString("\t\n!");
                    }else if((year == 18) || (year == 22) || (year == 26)){
                          g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x24,0xa1);
                          SerialPrintValue("18_B1 = ", B1, 10);
                          SerialPrintString("\t\n!");
                    }else if((year == 19) || (year == 23) || (year == 27)){
                          g_send_buff[g_con_read_flag] = Read_Add(0xa0,0x00,0x26,0xa1);
                          SerialPrintValue("19_B1 = ", B1, 10);
                          SerialPrintString("\t\n!");
                    }
                    g_con_read_flag++;
                    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 5 );
          }else{
                   g_total_datas = g_con_read_flag;
                   SerialPrintValue("g_total_datas = ", g_total_datas, 10);
                   SerialPrintString("\t\n!");
                   g_con_read_flag = 0;
                   aa = 0;
                   bb = 0;
                   cc = 0;
                   P0_7 = 0;
                   //SerialPrintString("444444444444444!\t\n!");
                   power_saving_in();   //??????
                   Disable_IIC_Ports();
                   g_check_evt = 1;
                   osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT);
                   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 100 );
                   //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, 10 );
          }
      }else{
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 800 );
      }
      return ( events ^ SBP_PERIODIC_CONN_READ_EVT );
  } 
  
  if ( events & SBP_PERIODIC_START_DATA_STORE_EVT )
  {
      uchar year = 0;
      static uchar cc = 0;
      if((P0_6 == 0) && ((g_is_busy == NO_EVT_RUNNING) || (g_is_busy == START_DATA_STORE_EVT)))
      {
          P0_7 = 1;
          g_is_busy = START_DATA_STORE_EVT;
          year = Read_Add(0xa0,0x00,0xE0,0xa1);
          SerialPrintValue("year = ", year, 10);
          SerialPrintString("\t\n!");
          //B1: 0x20   B2: 0x12    B3: 0x21
          if(cc == 0){
              if((year == 16) || (year == 20) || (year == 24)){
                    B1 = Read_Add(0xa0,0x00,0x20,0xa1);
                    SerialPrintValue("16_B1 = ", B1, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 17) || (year == 21) || (year == 25)){
                    B1 = Read_Add(0xa0,0x00,0x22,0xa1);
                    SerialPrintValue("16_B1 = ", B1, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 18) || (year == 22) || (year == 26)){
                    B1 = Read_Add(0xa0,0x00,0x24,0xa1);
                    SerialPrintValue("18_B1 = ", B1, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 19) || (year == 23) || (year == 27)){
                    B1 = Read_Add(0xa0,0x00,0x26,0xa1);
                    SerialPrintValue("19_B1 = ", B1, 10);
                    SerialPrintString("\t\n!");
              }
              cc++;
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, 5 );
              //return ( events ^ SBP_PERIODIC_START_DATA_STORE_EVT );
          }else if(cc == 1){
              B2 = Read_Add(0xa0,0x00,0x12,0xa1);
              SerialPrintValue("12_B2 = ", B2, 10);
              SerialPrintString("\t\n!");
              cc++;
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, 5 );
              //return ( events ^ SBP_PERIODIC_START_DATA_STORE_EVT );
          }else if(cc == 2){
              if((year == 16) || (year == 20) || (year == 24)){
                    B3 = Read_Add(0xa0,0x00,0x21,0xa1);
                    SerialPrintValue("16_B3 = ", B3, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 17) || (year == 21) || (year == 25)){
                    B3 = Read_Add(0xa0,0x00,0x23,0xa1);
                    SerialPrintValue("17_B3 = ", B3, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 18) || (year == 22) || (year == 26)){
                    B3 = Read_Add(0xa0,0x00,0x25,0xa1);
                    SerialPrintValue("18_B3 = ", B3, 10);
                    SerialPrintString("\t\n!");
              }else if((year == 19) || (year == 23) || (year == 27)){
                    B3 = Read_Add(0xa0,0x00,0x27,0xa1);
                    SerialPrintValue("19_B3 = ", B3, 10);
                    SerialPrintString("\t\n!");
              }
              cc++;
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, 5 );

          }else if(cc == 3){
              cc = 0;
              //P0_7 = 0;
              osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT );
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_B1_3_STORE_EVT, 5 );
          
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, (uint32)10*60*1000 );
          }
      }else{
          SerialPrintString("P0_6 = 1xxxxx\t\n!");
          SerialPrintValue("P0_6 = ", P0_6, 10);
          SerialPrintValue("P0_7 = ", P0_7, 10);
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_START_DATA_STORE_EVT, 800 );
      }
      


      return ( events ^ SBP_PERIODIC_START_DATA_STORE_EVT );
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

//END
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
      //simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}


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
        P0_7 = 0;
        g_is_busy = NO_EVT_RUNNING;
        //ready_to_notify = 0;
        Disable_IIC_Ports();
        power_saving_in();
      }
      break;

    case GAPROLE_CONNECTED:
      {
          SerialPrintString("CONNECTED!!!!\t\n!");
          power_saving_out();
          HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_ON );
          ready_to_notify = 1;
          g_con_read_flag = 0;
          //P0_6 = 0;
          //Write_Add(0xa0,0x00,0x01,0x66);//向器件写一个数据: 
          P0_7 = 0;
          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CONN_READ_EVT, 500 );
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
          SerialPrintString("xxxxDISCONNECT!!!!\t\n!");
          pre_lens = 0;
          ready_to_notify = 0;
          g_con_read_flag = 0;
          P0_7 = 0;
          g_check_evt=0;
          g_is_busy = NO_EVT_RUNNING;
          Disable_IIC_Ports();
          power_saving_in();
          //advertising_enable = FALSE;
          //GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        P0_7 = 0;
        ready_to_notify = 0;
        Disable_IIC_Ports();
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
/*
static void performPeriodicTask( void )
{

}
*/


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
  uint8 lngs;

  uint8 cmp_str[] = {0x56, 0x78, 0x9A, 0xBC};
  power_saving_out();
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:      
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newStr );
      //lngs = strlen(newStr);
      //SerialPrintValue("strlen = ", lngs, 10);
      SerialPrintString("qqqqqqqqqqqqq\t\n");
      if((strlen((char *)newStr) >= 1))
      {

          if(osal_memcmp(newStr, cmp_str, 4)){   //(newStr[0] == 0x65) && (newStr[1] == 0x78) && (newStr[0] == 0x9A) && (newStr[1] == 0xBC)
              //传输温度图表数据
              SerialPrintString("$$$$$$$$$$$ notify temperature data $$$$$$$$$$$$$$!\t\n");
              g_check_evt = 2;
              osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_CHECK_EVT, 100 );
          }else{

             if((newStr[0] == 0x41) && (newStr[1] == 0x52))
             {
                  SerialPrintString("start to store data!\t\n");
                  g_start_data = 1;
             }else if((newStr[0] == 0x63) && (newStr[1] == 0x74)){
                  SerialPrintString("end to store data!\t\n");
	              g_rsv_total_datas = pre_lens;
	              SerialPrintValue("g_rsv_total_datas = ", g_rsv_total_datas, 10);
	              SerialPrintString("\t\n!");
                  g_set_write_flag = 0;
                  g_start_data = 0;
                  pre_lens = 0;
                  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_IIC_WRITE_PERMIT_EVT, 1000 );
             }else if(g_start_data == 1)
             {
                  lngs = newStr[0] - 1;
                  SerialPrintValue("lngs = ", lngs, 10);
                  SerialPrintString("\t\n");
                  osal_memcpy(&g_rsv_buff[pre_lens], &newStr[1], lngs);
                  pre_lens = pre_lens + lngs;
                  if(pre_lens > 218){
                      pre_lens = 0;
                  }
             }
          }
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
