#include "main.h"
#include "math.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/

void USART3_Configuration(void)
{
		USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 

		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);

		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

		USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
		USART_Cmd(USART3,ENABLE);

		nvic.NVIC_IRQChannel = USART3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 3;
		nvic.NVIC_IRQChannelSubPriority = 3;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
}

void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
			USART_SendData(USART3,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}	

float send_data[4];
void shanwai_send_data1(uint8_t *value,uint32_t size )
{
	USART3_SendChar(0x03);
	USART3_SendChar(0xfc);
	while(size)
	{
		USART3_SendChar(*value);
		value++;
		size--;
	}
	USART3_SendChar(0xfc);
	USART3_SendChar(0x03);
}

//							send_data[0] = target_speed*1000.0;
//							send_data[1] = current_speed*1000.0;
//							send_data[2] = Moto_Pid4.ESC_output_speed;
//							send_data[3] = Error;

//							shanwai_send_data1((uint8_t*)&send_data,sizeof(send_data));

union _Speed_
{
	unsigned char Serial_buf[12];     //接收缓冲,最大USART_REC_LEN个字节.
	struct _L_R_speed_
	{
		unsigned int flag;
		float Right_Speed;
		float Left_Speed;
	}Struct_Speed;
}Union_Reciver;
u16 USART_RX_STA=0; 


char Res;
extern float target_speed[5];
unsigned int buzzer_flag = 0;

void RGB_Led()
{
	if(target_speed[1] > target_speed[2])
		RGB_Led_Control(1,2);
	else if(target_speed[1] < target_speed[2])
		RGB_Led_Control(2,1);
	else if(target_speed[1] < 0 && target_speed[2] < 0)
		RGB_Led_Control(2,2);
	else
		RGB_Led_Control(1,1);
}
void USART3_IRQHandler(void)                	
{
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
		Union_Reciver.Serial_buf[USART_RX_STA] = Res;
		if(USART_RX_STA == 12)
			USART_RX_STA = 0;
		USART_RX_STA++;
		
		if(Union_Reciver.Struct_Speed.flag == 255)
		{
				buzzer_flag ++;
				target_speed[1] = target_speed[4] = Union_Reciver.Struct_Speed.Right_Speed;
				target_speed[2] = target_speed[3] = Union_Reciver.Struct_Speed.Left_Speed;
				if(buzzer_flag < 5000)  //上位机连接ZXBOT正常时报警
					BUZZER_ON();  //ON
				else
				{
					BUZZER_OFF();
					buzzer_flag = 6000;
				}
				RGB_Led_Control(1,1);
		}
		else
		{
				BUZZER_OFF();
				RGB_Led_Control(3,3);
				USART_RX_STA = 0;
		}
	} 
}
		
union _Send_Data
{
		signed char Data_Buf[16];
		struct _Encoder_
		{
				float FLag_Float;            //填充小数字头
				signed int Right_Encoder;    //右轮的编码器累加值，经过处理后的数据
 				signed int Left_Encoder;     //左轮的编码器累加值，经过处理后的数据
				signed int Magnetometer;       //磁力计的YAW轴数据
		}Struct_Encoder;
}Union_Send_Data;

/*********************向上位机发送编码器的累加值********************************/
void Usart_Send_Char(unsigned int Right_value,unsigned int Left_value,unsigned int Magnet_value)  
{
		unsigned char i = 0;
		Union_Send_Data.Struct_Encoder.FLag_Float  = 1.5;
		Union_Send_Data.Struct_Encoder.Right_Encoder = Right_value;
		Union_Send_Data.Struct_Encoder.Left_Encoder  = Left_value;
		Union_Send_Data.Struct_Encoder.Magnetometer  = Magnet_value;
		
		for(i=0;i<16;i++)
		{
				USART3_SendChar(Union_Send_Data.Data_Buf[i]);
		}
}
