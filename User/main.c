#include "main.h"


void HARD_WARE_init()
{
	TIM6_Configuration();
	Led_Configuration();                //LED Configuration init.
	Buzzer_Configuration();             //beep Configuration init.
	CAN1_Configuration();               //CAN1 Configuration init.
	CAN2_Configuration();               //CAN2 Configuration init.
	USART2_Configuration();             //usart2 and DMA Configuration init.
	USART3_Configuration();             //usart3 Configuration init.
	RGBLed_Configuration();

}


#define MOTO_ENCODER_MAX_VALUE                    8192      //����һȦ�������ķ���ֵ
#define MOVE_1PERIMETER                           19645.0f  //1�ױ��������ۼ�ֵ
#define PID_TIMES                                 0.0104    //s
#define ESC_MAX                                   100000
#define DIMENSION_VALUE                           1000      //���ٱ仯����
#define DEAD_ZONE                                 300       //�����������

unsigned int Time_Update = 0;  //��������ʱ���趨5.2ms �ٵ��İ���ʱ��������
extern signed int Encoder_Value[5],Last_Encoder_Value[5];   //���������ۼ�ֵ ���ǳ�С

float SubRal_Encoder_Speed[5]={0.0};  //��ŵ����ʵ��ת��ֵ
int8_t flag = 0;

struct PID_PAR_value  //����ʽPID���ݴ洢��
{
		float Error_Diff; 
		float L_Error;
		float LL_Error;
		signed int ESC_output_speed;
		unsigned int OutPut_Add;
}Moto_Pid1,Moto_Pid2,Moto_Pid3,Moto_Pid4;


void PID_Control_Function(float current_speed,float target_speed,unsigned char moto_id)
{
		static float l_p  =  1.1;   //1.1
		static float l_i  =  0.02;   //0.02
		static float l_d  =  0.08;   //0.08
	
		float Error = 0.0;
		float D_Error = 0.0;

		
		Error  =  target_speed-current_speed;   //���㵱ǰƫ��ֵ
			switch(moto_id)
			{
					case 1:
					{
							Moto_Pid1.Error_Diff    +=  Error;
							D_Error       =  	Error - Moto_Pid1.L_Error;
							Moto_Pid1.ESC_output_speed  = (Error*l_p + Moto_Pid1.Error_Diff*l_i + D_Error*l_d) * DIMENSION_VALUE;  //pid���ֵ����
							Moto_Pid1.L_Error=Error;
							if(Moto_Pid1.ESC_output_speed > 200)
									Moto_Pid1.ESC_output_speed  = Moto_Pid1.ESC_output_speed + DEAD_ZONE;
							else if(Moto_Pid1.ESC_output_speed < -200)
									Moto_Pid1.ESC_output_speed  = Moto_Pid1.ESC_output_speed - DEAD_ZONE;
							else
								Moto_Pid1.ESC_output_speed  = Moto_Pid1.ESC_output_speed;
					}break;
					
					case 2: 
					{
							Moto_Pid2.Error_Diff    +=  Error;
							D_Error       =  	Error - Moto_Pid2.L_Error;
							Moto_Pid2.ESC_output_speed  = (Error*l_p + Moto_Pid2.Error_Diff*l_i + D_Error*l_d) * DIMENSION_VALUE;  //pid���ֵ����
							Moto_Pid2.L_Error=Error;	
						
							if(Moto_Pid2.ESC_output_speed > 200)
									Moto_Pid2.ESC_output_speed  = Moto_Pid2.ESC_output_speed + DEAD_ZONE;
							else if(Moto_Pid1.ESC_output_speed < -200)
									Moto_Pid2.ESC_output_speed  = Moto_Pid2.ESC_output_speed - DEAD_ZONE;
							else
								Moto_Pid2.ESC_output_speed  = Moto_Pid2.ESC_output_speed;
					}break;
					case 3:
					{
							Moto_Pid3.Error_Diff    +=  Error;
							D_Error       =  	Error - Moto_Pid3.L_Error;
							Moto_Pid3.ESC_output_speed  = (Error*l_p + Moto_Pid3.Error_Diff*l_i + D_Error*l_d) * DIMENSION_VALUE;  //pid���ֵ����
							Moto_Pid3.L_Error=Error;
						
							if(Moto_Pid3.ESC_output_speed > 200)
									Moto_Pid3.ESC_output_speed  = Moto_Pid3.ESC_output_speed + DEAD_ZONE;
							else if(Moto_Pid3.ESC_output_speed < -200)
									Moto_Pid3.ESC_output_speed  = Moto_Pid3.ESC_output_speed - DEAD_ZONE;
							else
								Moto_Pid3.ESC_output_speed  = Moto_Pid3.ESC_output_speed;
					}break;
					case 4:
					{
							Moto_Pid4.Error_Diff    +=  Error;
							D_Error       =  	Error - Moto_Pid4.L_Error;
							Moto_Pid4.ESC_output_speed  = (Error*l_p + Moto_Pid4.Error_Diff*l_i + D_Error*l_d) * DIMENSION_VALUE;  //pid���ֵ����
							Moto_Pid4.L_Error=Error;
						
							(abs(Moto_Pid4.ESC_output_speed)<100)?(Moto_Pid4.ESC_output_speed = 0) :(Moto_Pid4.ESC_output_speed = Moto_Pid4.ESC_output_speed);

						
							if(Moto_Pid4.ESC_output_speed > 200)
									Moto_Pid4.ESC_output_speed  = Moto_Pid4.ESC_output_speed + DEAD_ZONE;
							else if(Moto_Pid4.ESC_output_speed < -200)
									Moto_Pid4.ESC_output_speed  = Moto_Pid4.ESC_output_speed - DEAD_ZONE;
							else
								Moto_Pid4.ESC_output_speed  = Moto_Pid4.ESC_output_speed;
					}break;
				}
}

extern signed int ZGyroModuleAngle; //�����Ƶ�ԭʼֵ
float target_speed[5];  //ȫ��ģʽ����ĸ����ٶ�ֵ����λm/s

unsigned  int Count_Time = 0;   //�����������
int main(void)
{     
		HARD_WARE_init();
		TIM6_Start();
		
		while(1)
		{
				if(Time_Update==1)  //��������5.2MS �����������ܵİְ� �����޸ĵ�Ŷ
				{
						Count_Time ++;   //���������
					
						if(Count_Time < 300) //���ǻ�����ë�� �����ص���ʼλ�ã������𵴣�����ǰ����Ҫ����������λ�����
						{
								for(flag=1;flag<5;flag++)
										Last_Encoder_Value[flag] = Encoder_Value[flag];
								BUZZER_ON();   //״̬��ʼ������
								RGB_Led_Control(2,2);
						}
						else  //�����������󣬽���״̬����
						{
								BUZZER_OFF();    //״̬�����رձ���
								for(flag=1;flag<5;flag++)
								{
										SubRal_Encoder_Speed[flag] = (float)(Encoder_Value[flag] - Last_Encoder_Value[flag]); //��ȡ�������Ĳ�ֵ
										Last_Encoder_Value[flag] = Encoder_Value[flag];

										SubRal_Encoder_Speed[flag] = 	SubRal_Encoder_Speed[flag]/MOVE_1PERIMETER*192.3f;	 //����ʵ��ת��ֵm/s

										if(flag == 1 || flag == 4)
												PID_Control_Function(-SubRal_Encoder_Speed[flag],target_speed[flag],flag);  //����PID����
										else
												PID_Control_Function(SubRal_Encoder_Speed[flag],target_speed[flag],flag);
								}
								Set_CM_Speed(-Moto_Pid1.ESC_output_speed,Moto_Pid2.ESC_output_speed,Moto_Pid3.ESC_output_speed,-Moto_Pid4.ESC_output_speed); 
								Usart_Send_Char(((Encoder_Value[1]+Encoder_Value[4])/2),((Encoder_Value[2]+Encoder_Value[3])/2),ZGyroModuleAngle);  // ���ͱ�����������
								RGB_Led();
								Count_Time = 1000;
							}
						Time_Update=0;
				}		
			}
}
