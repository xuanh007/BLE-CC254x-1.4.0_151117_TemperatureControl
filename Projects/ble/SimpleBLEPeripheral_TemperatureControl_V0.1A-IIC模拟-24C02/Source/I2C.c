//----------------------------------------------------------------------------- 
// Includes 

//----------------------------------------------------------------------------- 

//#include "I2C.h"
#include <ioCC2541.h>
/* ------------------------------------------------------------------------------------------------
                           Constants
 * ------------------------------------------------------------------------------------------------  */ 

// I2CWC 

#define I2C_OVR            BV(7) // 1: GPIO functionality.0: I2C functionality 
#define I2C_SCLPUE         BV(3) //SCL pin pullup enable 
#define I2C_SDAPUE         BV(2) //SDA pin pullup enable. 
#define I2C_SCLOE          BV(1) //SCL pin output enable 
#define I2C_SDAOE          BV(0) //SDA pin output enable 
 
// I2CIO 

#define I2C_SCLD             BV(1) //SCL data value
#define I2C_SDAD             BV(0) //SDA data value  

#define SDA_0      I2CIO &= ~I2C_SDAD //SDA=0 
#define SDA_1      I2CIO |= I2C_SDAD  //SDA=1 
#define SCL_0      I2CIO &= ~I2C_SCLD //SCL=0 
#define SCL_1      I2CIO |= I2C_SCLD  //SCL=1  

#define SDA_IN      I2CWC &= ~I2C_SDAOE //SDA INPUT 
#define SDA_OUT     I2CWC |= I2C_SDAOE  //SDA OUTPUT 
#define SCL_IN      I2CWC &= ~I2C_SCLOE //SCL INPUT 
#define SCL_OUT     I2CWC |= I2C_SCLOE  //SCL OUTPUT  
#define I2C_GPIO  I2CWC = 0x80; //1: I2C GPIO  

#define uint8	unsigned char
#define uint32  unsigned int
//*********************************************************************************** 

//*名称：i2c_send_noack()                          *
 
//*功能：无应答I2C总线           * 
//*参数：无           * 
//*返回：1 有NOACK信号 0 无NOACK信号

//*********************************************************************************** 
void send_noack()
{  
	SDA_OUT; //开路输出,  
	SDA_1;  //SDA = 1; NO ACK  
	asm("nop"); 
	SCL_1;  //SCL = 1;
	asm("nop");  
	SCL_0;  //SCL = 0; //START
 } 

// iic应答  for slaver 
void send_ack() { 
 
	SDA_OUT; //开路输出,  
	SDA_0; //OUT 0 ACK  
	asm("nop");  
	SCL_1;  
	sm("nop");  
	SCL_0; 
}  
/* 
 *停止iic  */ 

static void stop(){ 
 
	SDA_OUT; //开路输出,并输出0.  
	SCL_0;  //SCL = 0;  
	asm("nop");  
	SCL_1;  //SCL = 1; STOP  
	asm("nop");  
	SDA_1;  //SDA = 1;
	SDA_IN;
	SCL_IN; 
}  
/* 
 * 启动iic  */ 
// static void start() { 
static void start(){  
	SDA_OUT; //开路输出,并输出0.         
	SCL_OUT;  
	SDA_1;  //SDA = 1;  
	SCL_1;  //SCL = 1;  
	asm("nop"); 
	SDA_0;  //SDA = 0;  
	asm("nop");  
	SCL_0;  //SCL = 0; //START 
}  
/* 
 * iic写一个字节  */ 

void iic_write(uint8 datIn) {  
	uint8 dat, j;  
	dat = datIn;         
	SDA_OUT;  
	for (j = 0; j < 8; j++) { 
		if((dat& 0x80)) SDA_1; 
		else SDA_0; 

		asm("nop");asm("nop"); asm("nop");   
		SCL_1; //write TDOS_SDA begin 
		asm("nop");   
		dat<<= 1;   
		SCL_0; //write TDOS_SDA end  
		} 
} 

 
unsigned char check_ack() {  
	unsigned char ack_flag;   
	SDA_IN; //开路输入,  
	asm("nop");  
	SCL_1; //read ask begin  
	asm("nop");   

	if((I2CIO & I2C_SDAD) ==1){ //if (SDA==1)   
		ack_flag = 0; //1: err  
	}else{   
		ack_flag = 1; //0: ok  
	}  
	SCL_0; //read ask end  
	return ack_flag;
 } 
 /* 
 * iic读一个字节  */ 

uint8 iic_read() {
	uint8 j, dat = 0; 
	 
	SDA_IN; //开路输入,  
	for (j = 0; j < 8; j++) {   
		SCL_1; //read TDOS_SDA begin. delay 0.7us   
		dat<<= 1; 

		asm("nop"); asm("nop"); asm("nop"); 
		if((I2CIO & I2C_SDAD) ==1){ //if (SDA==1)    
		dat |= 0x01; //input TDOS_SDA   
		}   
		SCL_0; //read TDOS_SDA end. delay 1.4us  
	}  
	return dat; 
} 
/* 
 * 通过I2C总线向某一寄存器写入一个字节数据  */ 
void I2C_write_byte(uint8 I2C_addr, uint8 Raddr, uint8 dat) {  
	start(); //发启动信号 
	 
	iic_write(I2C_addr | 0X00); //WRITE i2c  
	if (check_ack()==0) {   
		goto err;  
	} 

	iic_write(Raddr); //发送寄存器地址  
	if (check_ack()==0) {   
		goto err;  
	}
	
	iic_write(dat); //发送数据字节  
	if (check_ack()==0) {   
		goto err;  
	} 
	err:  
		stop(); 
}  
/* 
 * 通过I2C总线读出某一寄存器的数据  */ 

uint8 I2C_read_byte(uint8 I2C_addr, uint8 Raddr) {  
	uint8 dat;   
	start();  
	iic_write(I2C_addr | 0X00); //WRITE i2c  
	if (check_ack()==0) {   
		goto err; 
	} 
 
	iic_write(Raddr); //TDOS'register  
	if (check_ack()==0) {   
		goto err;  
	} 
 
	start(); //重发起动信号 
 
	iic_write(I2C_addr | 0X01); //READ  
	if (check_ack()==0) {   
		goto err;  
	} 
 
	dat = iic_read(); //接收读出的数据 
	send_noack(); 
	err:  
		stop();
	return dat; 
} 
/* 
 * 通过I2C总线读出某一寄存器的数据  */ 

uint32 I2C_read_3byte(uint8 I2C_addr, uint8 Raddr) {  
	uint8 dat1,dat2,dat3;   
	start();  
	iic_write(0X98 | 0X00); //WRITE i2c  
	if (check_ack()==0) {   goto err;  }  
	iic_write(0X00); //TDOS'register  
	if (check_ack()==0) {   goto err;  }  
	start(); //重发起动信号  
	iic_write(0X98 | 0X01); //READ  
	if (check_ack()==0) {   goto err;  }  
	dat1 = iic_read(); //接收读出的数据  
	send_ack();  
	dat2 = iic_read(); //接收读出的数据  
	send_ack();  
	dat3 = iic_read(); //接收读出的数据   
	send_noack(); 
	err:
		stop();  
	return (dat1<<16) | (dat2<<8) | dat3; 
} 

//----------------------------------------------------------------------------- 
// End Of File //---------------







