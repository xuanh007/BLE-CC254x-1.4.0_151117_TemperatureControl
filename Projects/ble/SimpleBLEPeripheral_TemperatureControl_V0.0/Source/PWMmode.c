#include <ioCC2540.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "PWMmode.h"

uint16 pwm_level[25][3] = {
//  {0,0,0},
  {2621,2621,2621},     //level 1
  {5242,5242,5242},     //level 2
  {7864,7864,7864},     //level 3
  {10485,10485,10485},  //level 4
  {13107,13107,13107},  //level 5
  {15728,15728,15728},  //level 6
  {18350,18350,18350},  //level 7
  {20971,20971,20971},  //level 8
  {23592,23592,23592},  //level 9
  {26214,26214,26214},  //level 10
  {28835,28835,28835},  //level 11
  {31457,31457,31457},  //level 12
  {34078,34078,34078},  //level 13
  {36700,36700,36700},  //level 14
  {39321,39321,39321},  //level 15
  {41943,41943,41943},  //level 16
  {44564,44564,44564},  //level 17
  {47185,47185,47185},  //level 18
  {49807,49807,49807},  //level 19
  {52428,52428,52428},  //level 20
  {55050,55050,55050},  //level 21
  {57671,57671,57671},  //level 22
  {60293,60293,60293},  //level 23
  {62914,62914,62914},  //level 24
  {65535,65535,65535},  //level 25
};

uint8 g_level = 0;
uint8 g_mode = 0;
uint8 g_ispwm = 0;
uint16 g_lowtime = 0;
uint16 g_hightime = 0;
uint8 g_is_vib_running = 0;
uint8 jyun_gpio_vib_is_running(void)
{
    return g_is_vib_running;
}

void jyun_set_vib_running_flag(uint8 duty)
{
    if(duty == 0){
       g_is_vib_running = 0;
	}else{
	   g_is_vib_running = 1;	
	}
}

void PWMmode_Init()
{
  //设置pwm端口为输出
//  P0DIR|= BV(6)|BV(7);
//  P1DIR|= BV(2);
  //设置pwm端口为外设端口，非gpio
//  P0SEL|= BV(6)|BV(7);
//  P1SEL|= BV(2);
  P0DIR|= BV(6)|BV(7);
  P0SEL|= BV(6)|BV(7);
  P1DIR|= BV(2);
  P1SEL|= BV(2);
  //由于uart等会占用我们当前使用的pwm端口，因此需要将uart等重映射到别的端口去。
  PERCFG |= 0x40;             // Move USART1&2 to alternate2 location so that T1 is visible
//  P2SEL |= BV(3);
//  P2SEL &= ~BV(4);

  // Initialize Timer 1
  T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended       
  T1CCTL0 = 0x24;
//  T1CCTL1 = 0x24;             // IM = 0; CMP = Clear output on compare; Mode = Compare
//  T1CCTL2 = 0x24;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL3 = 0x24;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CCTL4 = 0x24;             // IM = 0, CMP = Clear output on compare; Mode = Compare

  T1CNTL = 0x00;                 // Reset timer to 0;

//  T1CCTL0 = 0x4C;           
  T1CC0H = 0x00;             
  T1CC0L = 0x00;//77 1ms should be 125(dec)        18F             
//  T1CC1H = 0x00;             
//  T1CC1L = 0x00;
//  T1CC2H = 0x00;              
//  T1CC2L = 0x00;
  T1CC3H = 0x00;              
  T1CC3L = 0x00;  
  T1CC4H = 0x00;              
  T1CC4L = 0x00;  

  EA=1;
  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

void GPIOmode_init()
{
  //设置pwm端口为输出
  P0DIR|= BV(6)|BV(7);
  P1DIR|= BV(2);
  //设置pwm端口为gpio
  P0SEL&= 0x3F;
  P1SEL&= ~BV(2);
  //由于uart等会占用我们当前使用的pwm端口，因此需要将uart等重映射到别的端口去。
  PERCFG |= 0x40;             // Move USART1&2 to alternate2 location so that T1 is visible

  // Initialize Timer 1
  T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended          

  T1CCTL1 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL2 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL3 = 0x0C;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CCTL4 = 0x0C;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  
  T1CNTL = 0;                 // Reset timer to 0;

  T1CCTL0 = 0x4C;           
  T1CC0H = 0x00;             
  T1CC0L = 0xF9;//77 1ms should be 125(dec)           

  T1CC1H = 0x00;             
  T1CC1L = 0x00;
  T1CC2H = 0x00;              
  T1CC2L = 0x00;
  T1CC3H = 0x00;              
  T1CC3L = 0x00;  
  T1CC4H = 0x00;              
  T1CC4L = 0x00;  
  
  P0_6 = 0;
  P0_7 = 0;
  P1_2 = 0;
  
  EA=1;
  IEN1 |= 0x02;               // Enable T1 cpu interrupt
}

void PWM_Mode()
{

  T1CC0L = (uint8)pwm_level[g_level][0];
  T1CC0H = (uint8)(pwm_level[g_level][0] >> 8);
//  T1CC1L = (uint8)pwm_level[g_level][0];
//  T1CC1H = (uint8)(pwm_level[g_level][0] >> 8);
//  T1CC2L = (uint8)pwm_level[g_level][1];
//  T1CC2H = (uint8)(pwm_level[g_level][1] >> 8);
  T1CC3L = (uint8)pwm_level[g_level][2];
  T1CC3H = (uint8)(pwm_level[g_level][2] >> 8);
  T1CC4L = (uint8)pwm_level[g_level][2];
  T1CC4H = (uint8)(pwm_level[g_level][2] >> 8);

  // Reset timer
  T1CNTL = 0x00;
  // Start timer in modulo mode.
  T1CTL &= 0xFC; 
  T1CTL |= 0x01; 
}
void PWMmode_SetLevel(uint8 level)
{
  if((level < 1) || (level > 25))
    return ;
  PWMmode_Init();
  g_level = level-1;
  g_ispwm = 1;
  PWM_Mode();
}

void GPIOSet_ext(uint8 status)
{
  if(status)
  {
    P0_6 = 1;
    P0_7 = 1;
    P1_2 = 1;
  }
  else{
    P0_6 = 0;
    P0_7 = 0;
    P1_2 = 0;
  }
}

void GPIO_fake_PWM()
{
  static uint16 msec = 0;
  static uint8 gpiostatus = 0;
  if(g_hightime == 0)
  {
    GPIOSet_ext(0);
    return;
  }
  if(g_lowtime == 0)
  {
    GPIOSet_ext(1);
    return;
  }
    msec++;
  
  if(gpiostatus)
  {
    if(msec >= g_hightime )
    {
      gpiostatus = 0;
      GPIOSet_ext(0);
      msec = 0;
    }
  }
  else
  {
    if(msec >= g_lowtime )
    {
      gpiostatus = 1;
      GPIOSet_ext(1);
      msec = 0;
    }
  }
    // Reset timer
  T1CNTL = 0;
  // Start timer in modulo mode.
  T1CTL &= 0xFC; 
  T1CTL |= 0x02; 
}

void GPIO_Mode()
{
  static uint16 msec = 0;
  static uint8 gpiostatus = 0;
  msec++;
  switch(g_mode)
  {
        case 0:
                if(gpiostatus)
                {
                        if(msec >= 1)
                        {
                                gpiostatus = 0;
                                GPIOSet_ext(0);
                                msec = 0;
                        }
                }
                else
                {
                        if(msec >= 2)
                        {
                                gpiostatus = 1;
                                GPIOSet_ext(1);
                                msec = 0;
                        }
                }
                break;
        case 1:
                if(gpiostatus)
                {
                        if(msec >= 80)
                        {
                                gpiostatus = 0;
                                GPIOSet_ext(0);
                                msec = 0;
                        }
                }
                else
                {
                        if(msec >= 60)
                        {
                                gpiostatus = 1;
                                GPIOSet_ext(1);
                                msec = 0;
                        }
                }
                break;
        case 2:
                if(gpiostatus)
                {
                        if(msec >= 400)
                        {
                                gpiostatus = 0;
                                GPIOSet_ext(0);
                                msec = 0;
                        }
                }
                else
                {
                        if(msec >= 400)
                        {
                                gpiostatus = 1;
                                GPIOSet_ext(1);
                                msec = 0;
                        }
                }
                break;
        case 3:
                if(gpiostatus)
                {
                        if(msec >= 240)
                        {
                                gpiostatus = 0;
                                GPIOSet_ext(0);
                                msec = 0;
                        }
                }
                else
                {
                        if(msec >= 440)
                        {
                                gpiostatus = 1;
                                GPIOSet_ext(1);
                                msec = 0;
                        }
                }
                break;
        case 4:
                if(gpiostatus)
                {
                        if(msec >= 40)
                        {
                                gpiostatus = 0;
                                GPIOSet_ext(0);
                                msec = 0;
                        }
                }
                else
                {
                        if(msec >= 80)
                        {
                                gpiostatus = 1;
                                GPIOSet_ext(1);
                                msec = 0;
                        }
                }
                break;
        case 5:
                gpiostatus = 1;
                GPIOSet_ext(1);
                msec = 0;
                break;
        case 6:
                gpiostatus = 0;
                GPIOSet_ext(0);
                msec = 0;
                break;
  }
  
  // Reset timer
  T1CNTL = 0;
  // Start timer in modulo mode.
  T1CTL |= 0x02; 
}
void GPIOmode_SetMode(uint8 mode)
{
  if( mode > 6 )
    return ;
  GPIOmode_init();
  g_mode = mode;
  g_ispwm = 0;
  GPIO_Mode();
}

void GPIO_fake_PWM_SetMode(uint16 period, uint8 duty)
{
  GPIOmode_init();
  jyun_set_vib_running_flag(duty);
  if(duty > 100){
	 g_hightime = period;
  }else{
     g_hightime = period * duty / 100;
  }
  g_lowtime = period - g_hightime;
  g_ispwm = 0;
  GPIO_fake_PWM();
}

//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) {
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01){    
      if(g_ispwm)
        PWM_Mode();  
      else GPIO_fake_PWM();//GPIO_Mode();
    }
    T1STAT = ~ flags;
}
