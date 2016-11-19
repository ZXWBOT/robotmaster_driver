#ifndef __ENCODER_H__
#define __ENCODER_H__

void RGBLed_Configuration(void);
void RGB_Led_Control(unsigned char LEFT_LED,unsigned char Right_LED);

#define LEFT_LED_RAD_COLOR       1 
#define LEFT_LED_BULE_COLOR      2  
#define LEFT_LED_PURPLE_COLOR    3  

#define RIGHT_LED_RAD_COLOR      1  
#define RIGHT_LED_BULE_COLOR     2  
#define RIGHT_LED_PURPLE_COLOR   3  

void RGB_Led(void);


#endif 
