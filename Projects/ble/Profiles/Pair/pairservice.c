/**************************************************************************************************
  Filename:       Pairservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Pair service.

  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_adc.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
//#include "hiddev.h"

#include "Pairservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define PAIR_STATUS_VALUE_IDX        2


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Pair service
CONST uint8 PairServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PAIR_SERV_UUID), HI_UINT16(PAIR_SERV_UUID)
};

// Pair level characteristic
CONST uint8 PairLevelUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PAIR_STATUS_UUID), HI_UINT16(PAIR_STATUS_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Application callback
static PairServiceCB_t PairServiceCB;

// Measurement setup callback
static PairServiceSetupCB_t PairServiceSetupCB = NULL;

// Measurement teardown callback
static PairServiceTeardownCB_t PairServiceTeardownCB = NULL;

// Measurement calculation callback
static PairServiceCalcCB_t PairServiceCalcCB = NULL;


// Critical Pair level setting
static uint8 PairCriticalLevel;

// ADC channel to be used for reading
static uint8 PairServiceAdcCh = HAL_ADC_CHANNEL_4;

/*********************************************************************
 * Profile Attributes - variables
 */

// Pair Service attribute
static CONST gattAttrType_t PairService = { ATT_BT_UUID_SIZE, PairServUUID };

// Pair level characteristic
static uint8 PairStatusProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 PairStatue = 0;
static gattCharCfg_t PairLevelClientCharCfg[GATT_MAX_NUM_CONN];


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t PairAttrTbl[] =
{
  // Pair Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&PairService                     /* pValue */
  },

    // Pair Level Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &PairStatusProps
    },

      // Pair Level Value
      {
        { ATT_BT_UUID_SIZE, PairLevelUUID },
        GATT_PERMIT_READ,
        0,
        &PairStatue
      },

      // Pair Level Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &PairLevelClientCharCfg
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 PairReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t PairWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset );
static void PairNotifyCB( linkDBItem_t *pLinkItem );
static void PairNotifyLevel( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Pair Service Callbacks
CONST gattServiceCBs_t PairCBs =
{
  PairReadAttrCB,  // Read callback function pointer
  PairWriteAttrCB, // Write callback function pointer
  NULL             // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Pair_AddService
 *
 * @brief   Initializes the Pair Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Pair_AddService( void )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, PairLevelClientCharCfg );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( PairAttrTbl,
                                        GATT_NUM_ATTRS( PairAttrTbl ),
                                        &PairCBs );

  return ( status );
}


bStatus_t PairSendStatus(uint8 level)
{
    // Update level
    PairStatue = level;

    // Send a notification
    PairNotifyLevel();
  return SUCCESS;
}



/*********************************************************************
 * @fn          PairReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 PairReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
  status = ATT_ERR_ATTR_NOT_FOUND;
  return ( status );
}

/*********************************************************************
 * @fn      PairWriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t PairWriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset )
{
	{
	  bStatus_t status = SUCCESS;
	
	  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
	  switch ( uuid )
	  {
		case GATT_CLIENT_CHAR_CFG_UUID:
		  status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
												   offset, GATT_CLIENT_CFG_NOTIFY );
		  if ( status == SUCCESS )
		  {
			uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
	
		  }
		  break;
	
		default:
		  status = ATT_ERR_ATTR_NOT_FOUND;
		  break;
	  }
	
	  return ( status );
	}

}

/*********************************************************************
 * @fn          PairNotifyCB
 *
 * @brief       Send a notification of the level state characteristic.
 *
 * @param       connHandle - linkDB item
 *
 * @return      None.
 */
static void PairNotifyCB( linkDBItem_t *pLinkItem )
{
  if ( pLinkItem->stateFlags & LINK_CONNECTED )
  {
    uint16 value = GATTServApp_ReadCharCfg( pLinkItem->connectionHandle,
                                            PairLevelClientCharCfg );
    if ( value & GATT_CLIENT_CFG_NOTIFY )
    {
      attHandleValueNoti_t noti;

      noti.handle = PairAttrTbl[PAIR_STATUS_VALUE_IDX].handle;
      noti.len = 1;
      noti.value[0] = PairStatue;

      GATT_Notification( pLinkItem->connectionHandle, &noti, FALSE );
    }
  }
}

/*********************************************************************
 * @fn      PairNotifyLevelState
 *
 * @brief   Send a notification of the Pair level state
 *          characteristic if a connection is established.
 *
 * @return  None.
 */
static void PairNotifyLevel( void )
{
  // Execute linkDB callback to send notification
  linkDB_PerformFunc( PairNotifyCB );
}


/*********************************************************************
*********************************************************************/
