#include <ioCC2540.h>
#define uchar unsigned char 
#define uint unsigned int 

#define sda P1_6;//;模拟I2C 数据 
#define scl P1_5;//;模拟I2C 时钟

void delay1(uint z)//延时为 1ms 
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

void delay()//5us延时 
{ 
_nop_(); 
_nop_(); 
_nop_(); 
}

void star()//开始 
{ 
sda=1; 
delay();//5us延时 
scl=1; 
delay();//5us延时 
sda=0; 
delay();//5us延时 

}

void stop()//停止 
{ 
sda=0; 
delay();//5us延时 
scl=1; 
delay();//5us延时 
sda=1; 
delay();//5us延时 

}

void ack()//应答 
{ uchar z=0; 
while((sda==1)&&(z<50))z++;//条件判断， sda=1，则没有应答。如果没有应答则延时：z<50，z++;后返回 
scl=0; 
delay();//5us延时 
}

///写一个数据函数 
//器件写地址 slave_write_address 
//字节高地址 byte_High_address 
//字节低地址 byte_Low_address 
//待写入数据 data_data 
void write(uchar slave_write_address,uchar byte_High_address,uchar byte_Low_address,uchar data_data)//写一个数据 
{ 
	uchar temp,temp1,i,ii;

	star();//开始

	for(ii=0;ii<4;ii++)//根据 24CXX文档资料，和时序图，按顺序送：器件写地址，字节地址，数据 
	{ 
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
		for(i=0;i<8;i++) 
		{ 
			scl=0; 
			delay();//5us延时 
			temp=temp1; 
			temp=temp&0x80;// 相 与 后，把不相关的位清零

			if(temp==0x80)//根据前面 相 与 后，判断 temp是否等于0x80，是则该位为 1 

				sda=1; 
			else 
				sda=0; 

			delay();//5us延时 
			scl=1; 
			delay();//5us延时 
			scl=0; 
			delay();//5us延时 
			temp1=temp1<<1;//向左移出1位 

		} 
		sda=1; 
		delay();//5us延时 
		scl=1; 
		delay();//5us延时 
		ack(); 
	} 
	stop();//停止 
}

///读一个数据函数 
//器件写地址 slave_write_address 
//器件读地址 slave_read_address 
//字节高地址 byte_High_address 
//字节低地址 byte_Low_address 
//读出的数据 data_data

read(uchar slave_write_address,byte_High_address,byte_Low_address,uchar slave_read_address)//读一个数据 
{ 
uchar temp,temp1,i,ii,x,data_data;

star();//开始

for(ii=0;ii<4;ii++)//根据 24CXX文档资料，和时序图，按顺序送：器件写地址，字节地址，器件读地址 
{ 
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
star();//开始

temp=slave_read_address;//送 器件读地址 
temp1=slave_read_address; 
}

for(i=0;i<8;i++)//开始读数据 
{ 
scl=0; 
delay();//5us延时 
temp=temp1; 
temp=temp&0x80;// 相 与 后，把不相关的位清零

if(temp==0x80)//根据前面 相 与 后，判断 temp是否等于0x80，是则该位为 1 

sda=1; 
else 
sda=0; 

delay();//5us延时 
scl=1; 
delay();//5us延时 
scl=0; 
delay();//5us延时 
temp1=temp1<<1;//向左移出1位 
} 
sda=1; 
delay();//5us延时 
scl=1; 
delay();//5us延时 
ack();//应答 
}

for(x=0;x<8;x++) 
{ 
data_data=data_data<<1;//向左移入1位

sda=1; 
delay();//5us延时 
scl=0; 
delay();//5us延时 
scl=1; 
delay();//5us延时

if(sda==1)//判断 数据线是否是高电平 
data_data|=0x01;//把读到的数据 或 0X01 
//else 
//data_data|=0x00; 
} 
ack();//应答 
stop();//停止 
return data_data;//返回读到的数据

}