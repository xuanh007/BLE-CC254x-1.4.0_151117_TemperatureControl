#ifndef _PWM_MODE_H_
#define _PWM_MODE_H_
/*
void PWMmode_SetLevel(uint8 level);
void GPIOmode_SetMode(uint8 mode);
*/
void GPIO_fake_PWM_SetMode(uint16 period, uint8 duty);
uint8 jyun_gpio_vib_is_running(void);
#endif