#include <ioCC2540.h>
#define uchar unsigned char 
#define uint unsigned int 

#define sda P1_6;//;ģ��I2C ���� 
#define scl P1_5;//;ģ��I2C ʱ��

void delay1(uint z)//��ʱΪ 1ms 
{ 
uchar x,x1; 
for(;z>0;z--) 
{ 
for(x=0;x<114;x++) 
{ 
for(x1=0;x1<1;x1++); 
} 
} 
}

void delay()//5us��ʱ 
{ 
_nop_(); 
_nop_(); 
_nop_(); 
}

void star()//��ʼ 
{ 
sda=1; 
delay();//5us��ʱ 
scl=1; 
delay();//5us��ʱ 
sda=0; 
delay();//5us��ʱ 

}

void stop()//ֹͣ 
{ 
sda=0; 
delay();//5us��ʱ 
scl=1; 
delay();//5us��ʱ 
sda=1; 
delay();//5us��ʱ 

}

void ack()//Ӧ�� 
{ uchar z=0; 
while((sda==1)&&(z<50))z++;//�����жϣ� sda=1����û��Ӧ�����û��Ӧ������ʱ��z<50��z++;�󷵻� 
scl=0; 
delay();//5us��ʱ 
}

///дһ�����ݺ��� 
//����д��ַ slave_write_address 
//�ֽڸߵ�ַ byte_High_address 
//�ֽڵ͵�ַ byte_Low_address 
//��д������ data_data 
void write(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar data_data)//дһ������ 
{ 
	uchar temp,temp1,i,ii;

	star();//��ʼ

	for(ii=0;ii<4;ii++)//���� 24CXX�ĵ����ϣ���ʱ��ͼ����˳���ͣ�����д��ַ���ֽڵ�ַ������ 
	{ 
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
		for(i=0;i<8;i++) 
		{ 
			scl=0; 
			delay();//5us��ʱ 
			temp=temp1; 
			temp=temp&0x80;// �� �� �󣬰Ѳ���ص�λ����

			if(temp==0x80)//����ǰ�� �� �� ���ж� temp�Ƿ����0x80�������λΪ 1 

				sda=1; 
			else 
				sda=0; 

			delay();//5us��ʱ 
			scl=1; 
			delay();//5us��ʱ 
			scl=0; 
			delay();//5us��ʱ 
			temp1=temp1<<1;//�����Ƴ�1λ 

		} 
		sda=1; 
		delay();//5us��ʱ 
		scl=1; 
		delay();//5us��ʱ 
		ack(); 
	} 
	stop();//ֹͣ 
}

///��һ�����ݺ��� 
//����д��ַ slave_write_address 
//��������ַ slave_read_address 
//�ֽڸߵ�ַ byte_High_address 
//�ֽڵ͵�ַ byte_Low_address 
//���������� data_data

read(uchar slave_write_address,byte_High_address,byte_Low_address,uchar slave_read_address)//��һ������ 
{ 
uchar temp,temp1,i,ii,x,data_data;

star();//��ʼ

for(ii=0;ii<4;ii++)//���� 24CXX�ĵ����ϣ���ʱ��ͼ����˳���ͣ�����д��ַ���ֽڵ�ַ����������ַ 
{ 
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
star();//��ʼ

temp=slave_read_address;//�� ��������ַ 
temp1=slave_read_address; 
}

for(i=0;i<8;i++)//��ʼ������ 
{ 
scl=0; 
delay();//5us��ʱ 
temp=temp1; 
temp=temp&0x80;// �� �� �󣬰Ѳ���ص�λ����

if(temp==0x80)//����ǰ�� �� �� ���ж� temp�Ƿ����0x80�������λΪ 1 

sda=1; 
else 
sda=0; 

delay();//5us��ʱ 
scl=1; 
delay();//5us��ʱ 
scl=0; 
delay();//5us��ʱ 
temp1=temp1<<1;//�����Ƴ�1λ 
} 
sda=1; 
delay();//5us��ʱ 
scl=1; 
delay();//5us��ʱ 
ack();//Ӧ�� 
}

for(x=0;x<8;x++) 
{ 
data_data=data_data<<1;//��������1λ

sda=1; 
delay();//5us��ʱ 
scl=0; 
delay();//5us��ʱ 
scl=1; 
delay();//5us��ʱ

if(sda==1)//�ж� �������Ƿ��Ǹߵ�ƽ 
data_data|=0x01;//�Ѷ��������� �� 0X01 
//else 
//data_data|=0x00; 
} 
ack();//Ӧ�� 
stop();//ֹͣ 
return data_data;//���ض���������

}