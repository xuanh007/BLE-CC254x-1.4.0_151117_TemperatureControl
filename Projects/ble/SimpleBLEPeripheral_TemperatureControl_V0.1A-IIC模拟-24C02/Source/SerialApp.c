#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hal_uart.h"



#include "PWMmode.h"
#include "SerialApp.h"

//static uint8 sendMsgTo_TaskID;

/*�ú����������������ĳ�ʼ�������е���*/
void SerialApp_Init( uint8 taskID )
{
  //����uart��ʼ������
  serialAppInitTransport();
  //��¼��������taskID������
//  sendMsgTo_TaskID = taskID;
}

/*uart��ʼ�����룬���ô��ڵĲ����ʡ������Ƶ�*/
void serialAppInitTransport( )
{
  halUARTCfg_t uartConfig;

  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = SBP_UART_BR;//������
  uartConfig.flowControl          = SBP_UART_FC;//������
  uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//��������ֵ��������flowControlʱ����������Ч
  uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart���ջ�������С
  uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart���ͻ�������С
  uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = SBP_UART_INT_ENABLE;//�Ƿ����ж�
  uartConfig.callBackFunc         = sbpSerialAppCallback;//uart���ջص��������ڸú����ж�ȡ����uart����

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

  return;
}
  uint16 numBytes;
/*uart���ջص�����*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
  uint8  pktBuffer[SBP_UART_RX_BUF_SIZE];
  uint8  compareBuffer1[]="test_BT";
  uint8  compareBuffer2[]="test_Pow";
  uint8  compareBuffer3[]="test_LED";  
  uint8  compareBuffer4[]="test_Mot";
  uint8  compareBuffer5[]="test_Key";
  uint8  compareBuffer6[]="pass_Mot";
  uint8  compareBuffer7[]="pass_LED";

  // unused input parameter; PC-Lint error 715.
  (void)event;
  int i=0;
  for(i=6000;i>0;i--){
	asm("nop");
  }
  //HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
  //���ؿɶ����ֽ�
  if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 ){
  	//��ȡȫ����Ч�����ݣ��������һ��һ����ȡ���Խ����ض�������
	(void)HalUARTRead (port, pktBuffer, numBytes);
        //���յ����ݺ������LCD��ʾ��
     if(osal_memcmp(pktBuffer,compareBuffer1,7)==TRUE){
         // SerialPrintString("test_bt!");		
     }else if(osal_memcmp(pktBuffer,compareBuffer2,8)==TRUE){

     }else if(osal_memcmp(pktBuffer,compareBuffer3,8)==TRUE){
       	HalLedSet( (HAL_LED_1 | HAL_LED_2 | HAL_LED_3 | HAL_LED_4 | HAL_LED_5), HAL_LED_MODE_ON); 
       // SerialPrintString("test1!");
  	 }else if(osal_memcmp(pktBuffer,compareBuffer4,8)==TRUE){

  	 }else if(osal_memcmp(pktBuffer,compareBuffer6,8)==TRUE){

  	 }else if(osal_memcmp(pktBuffer,compareBuffer7,8)==TRUE){
		HalLedSet( (HAL_LED_1 | HAL_LED_2 | HAL_LED_3 | HAL_LED_4 | HAL_LED_5), HAL_LED_MODE_OFF); 
  	 }
  }
  
}
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length)
{
	HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}
/*
��ӡһ���ַ���
str�����԰���0x00�����ǽ�β
*/
void SerialPrintString(uint8 str[])
{
  HalUARTWrite (SBP_UART_PORT, str, osal_strlen((char*)str));
}
/*
��ӡָ���ĸ�ʽ����ֵ
����
title,ǰ׺�ַ���
value,��Ҫ��ʾ����ֵ
format,��Ҫ��ʾ�Ľ��ƣ�ʮ����Ϊ10,ʮ������Ϊ16
*/
void SerialPrintValue(char *title, uint16 value, uint8 format)
{
  uint8 tmpLen;
  uint8 buf[256];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  SerialPrintString(buf);		
}
