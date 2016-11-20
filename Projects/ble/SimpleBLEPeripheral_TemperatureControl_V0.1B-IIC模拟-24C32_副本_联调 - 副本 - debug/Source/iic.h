#ifndef _IIC_H_
#define _IIC_H_

/***ucahr��uint �ĺ궨�����Ҫ����������ĺ����޷���������******/

#define uchar unsigned char   //����uchar������Ϊ�޷�����
#define uint unsigned int     //����uint������Ϊ�޷�����

#if 1
#define SCL P1_5                //����ģ��IIC��ʱ����
#define SDA P1_6                //����ģ��IIC��������
#else
#define SCL P0_7                //����
#define SDA P0_6                //����
#endif


struct read_data{
  uchar read[10];
};


/***�����ⲿ����****/
extern void delay_1ms(void);
extern void IIC_Init(void);                  //IIC��ʼ��
extern void Signal_Start(void);              //IICֹͣ�ź�
extern void Signal_Stop(void);              //IICֹͣ�ź�
extern void Disable_IIC_Ports(void);
extern void Write_Byte(uchar wdata);        //дһ���ֽ����ݺ���
extern uchar Read_Byte();                  //��һ���ֽ����ݺ���
extern void Write_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar data_data);    //��ĳ��IIC����дָ���ַ������
extern uchar Read_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address);               //��ĳ��IIC����дָ���ĳ����ַ���������
extern struct read_data Read_N_Add(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar slave_read_address);
#endif
