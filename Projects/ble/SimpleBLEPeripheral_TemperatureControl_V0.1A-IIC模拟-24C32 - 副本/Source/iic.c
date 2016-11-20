/****************************************************************************
* �� �� ��: main.c
* ��    ��: Amo [ www.amoMcu.com ��Ī��Ƭ��]
* ��    ��: 2014-04-08
* ��    ��: 1.0
* ��    ��: IIC io��ģ������
****************************************************************************/

#include <ioCC2540.h>
#include "iic.h"

#define  uchar unsigned  char 
#define  uint  unsigned  int
void IIC_Init(void)//IIC��ʼ��
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
void Signal_Start(void)//IIC��ʼ�ź�
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 0;
    delay_1ms(); 
}
void Signal_Stop(void)//IICֹͣ�ź�
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
    SDA = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 1;
    delay_1ms(); 
}
void Respons(void)//��Ӧ�ź�
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����
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
void No_Respons(void)//��Ӧ�ź�
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����
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

    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
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

    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����

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
    
    Signal_Start(); //����һ����ʼ�ź�
    
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//�� ����д��ַ 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//�� �ֽڸߵ�ַ 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//�� �ֽڵ͵�ַ 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                temp=data_data;//�� ���� 
                temp1=data_data; 
        } 
        Write_Byte(temp1);
        Respons();      //�ȴ���Ӧ
    }
    Signal_Stop();  //����һ����ֹ�ź�
}
uchar Read_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address)
{
    uchar temp,temp1,ii;
    uchar data_data;
    Signal_Start();     //����һ����ʼ�ź�
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//�� ����д��ַ 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//�� �ֽڸߵ�ַ 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//�� �ֽڵ͵�ַ 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                Signal_Start();//��ʼ

                temp=slave_read_address;//�� ��������ַ 
                temp1=slave_read_address; 
        }
        Write_Byte(temp1);
        Respons();          //�ȴ���Ӧ

    }
    data_data = Read_Byte();
    Signal_Stop();      //����һ����ֹ�ź�
    return data_data;       
}

struct read_data read_x;

struct read_data Read_N_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address)
{
    uchar temp,temp1,ii, i;
    //uchar data_data;
    Signal_Start();     //����һ����ʼ�ź�
    for(ii=0;ii<4;ii++){
        if(ii==0) 
        { 
                temp=slave_write_address;//�� ����д��ַ 
                temp1=slave_write_address; 
        } 
        else if(ii==1) 
        { 
                temp=byte_High_address;//�� �ֽڸߵ�ַ 
                temp1=byte_High_address; 
        } 
        else if(ii==2) 
        { 
                temp=byte_Low_address;//�� �ֽڵ͵�ַ 
                temp1=byte_Low_address; 
        } 
        else if(ii==3) 
        { 
                Signal_Start();//��ʼ

                temp=slave_read_address;//�� ��������ַ 
                temp1=slave_read_address; 
        }
        Write_Byte(temp1);
        Respons();          //�ȴ���Ӧ

    }
    for(i=0; i<9; i++){
        read_x.read[i] = Read_Byte();
        Respons();
    }
    if(i == 9){
        read_x.read[i] = Read_Byte();
        No_Respons();
    }
    Signal_Stop();      //����һ����ֹ�ź�
    return read_x;       
}


void delay_1ms(void)    //��� 0us  ��ʱ1ms
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
