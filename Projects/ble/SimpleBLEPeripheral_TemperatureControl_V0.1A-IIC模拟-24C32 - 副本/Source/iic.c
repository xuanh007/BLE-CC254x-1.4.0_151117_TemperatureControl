/****************************************************************************
* 文 件 名: main.c
* 作    者: Amo [ www.amoMcu.com 阿莫单片机]
* 修    订: 2014-04-08
* 版    本: 1.0
* 描    述: IIC io口模拟驱动
****************************************************************************/

#include <ioCC2540.h>
#include "iic.h"

#define  uchar unsigned  char 
#define  uint  unsigned  int
void IIC_Init(void)//IIC初始化
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
void Signal_Start(void)//IIC起始信号
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 0;
    delay_1ms(); 
}
void Signal_Stop(void)//IIC停止信号
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    SDA = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 1;
    delay_1ms(); 
}
void Respons(void)//答应信号
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入
    SDA = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SCL = 0;
    delay_1ms();
    
    if(i>=300)
    {
      delay_1ms();
    }
}
void No_Respons(void)//答应信号
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入
    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SCL = 0;
    delay_1ms();
    
    if(i>=300)
    {
      delay_1ms();
    }
}

void Write_Byte(uchar wdata)
{
    uchar i,mdata;

    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    mdata = wdata;
    for(i=0;i<8;i++)
    {        
        SCL = 0;
        delay_1ms();
        if(mdata & 0x80)
        {
          SDA = 1;
        }
        else
        {
          SDA = 0;
        }
        delay_1ms();
        SCL = 1;
        delay_1ms();  
        mdata <<= 1;
    }
    SCL = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
uchar Read_Byte()
{
    uchar i,rdata = 0;

    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入

    SCL = 0;
    delay_1ms();
    SCL = 1;
    for(i=0;i<8;i++)
    {
        SCL = 1;
        delay_1ms();
        rdata = (rdata<<1)|SDA;
        SCL = 0;
        delay_1ms();
    }
    return rdata;
}
void Write_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar data_data)
{
    uchar temp,temp1,ii;
    
    Signal_Start(); //产生一个起始信号
    
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//送 器件写地址 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//送 字节高地址 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//送 字节低地址 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                temp=data_data;//送 数据 
                temp1=data_data; 
        } 
        Write_Byte(temp1);
        Respons();      //等待答应
    }
    Signal_Stop();  //产生一个终止信号
}
uchar Read_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address)
{
    uchar temp,temp1,ii;
    uchar data_data;
    Signal_Start();     //产生一个起始信号
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//送 器件写地址 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//送 字节高地址 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//送 字节低地址 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                Signal_Start();//开始

                temp=slave_read_address;//送 器件读地址 
                temp1=slave_read_address; 
        }
        Write_Byte(temp1);
        Respons();          //等待答应

    }
    data_data = Read_Byte();
    Signal_Stop();      //产生一个终止信号
    return data_data;       
}

struct read_data read_x;

struct read_data Read_N_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address)
{
    uchar temp,temp1,ii, i;
    //uchar data_data;
    Signal_Start();     //产生一个起始信号
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//送 器件写地址 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//送 字节高地址 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//送 字节低地址 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                Signal_Start();//开始

                temp=slave_read_address;//送 器件读地址 
                temp1=slave_read_address; 
        }
        Write_Byte(temp1);
        Respons();          //等待答应

    }
    for(i=0; i<9; i++){
        read_x.read[i] = Read_Byte();
        Respons();
    }
    if(i == 9){
        read_x.read[i] = Read_Byte();
        No_Respons();
    }
    Signal_Stop();      //产生一个终止信号
    return read_x;       
}


void delay_1ms(void)    //误差 0us  延时1ms
{
    uchar a,b,c;    
    for(c=4;c>0;c--)
    {
        //for(b=142;b>0;b--)
        {
            for(a=2;a>0;a--)
            {
            }
        }
    }
}
