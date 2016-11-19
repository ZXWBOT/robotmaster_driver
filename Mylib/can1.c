#include "main.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_12tq;    //1 12  3  1
    can.CAN_BS2 = CAN_BS2_3tq;
    can.CAN_Prescaler = 1;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
	
    CAN_Transmit(CAN1,&tx_message);
}

void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

/***************************************
	*ZXW add CAN1 moto driver control function 
****************************************/
	
int8_t Moto_Flag_ID=0;                  //电机ID


volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
signed int Encoder_Value[5]={0},Last_Encoder_Value[5]={0};

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
			v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
			v->ecd_value = v->ecd_bias;
			v->last_raw_value = v->ecd_bias;
			v->temp_count++;
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
		v->last_raw_value = v->raw_value;
		v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
		v->diff = v->raw_value - v->last_raw_value;
		if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
				v->round_cnt++;
				v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff>7500)
		{
				v->round_cnt--;
				v->ecd_raw_rate = v->diff- 8192;
		}		
		else
		{
				v->ecd_raw_rate = v->diff;
		}
		//计算得到连续的编码器输出值
		v->ecd_value = v->raw_value + v->round_cnt * 8192;
		switch(Moto_Flag_ID)
		{
				case 1:Encoder_Value[1] = v->ecd_value;    break;
				case 2:Encoder_Value[2] = v->ecd_value;    break;
				case 3:Encoder_Value[3] = v->ecd_value;    break;
				case 4:Encoder_Value[4] = v->ecd_value;    break;
		}
}
static unsigned int can_count = 0;  //CAN总线电机透传数据计数
void CAN1_RX0_IRQHandler()
{
    CanRxMsg rx_message;
    can_count++;
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
					switch(rx_message.StdId)
					{
						case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:   Moto_Flag_ID = 1;
									(can_count<=50) ? GetEncoderBias(&CM1Encoder ,&rx_message):EncoderProcess(&CM1Encoder ,&rx_message);       //获取到编码器的初始偏差值 
									break;
						case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:   Moto_Flag_ID = 2;
									(can_count<=50) ? GetEncoderBias(&CM2Encoder ,&rx_message):EncoderProcess(&CM2Encoder ,&rx_message);
									break;
						case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:   Moto_Flag_ID = 3;
									(can_count<=50) ? GetEncoderBias(&CM3Encoder ,&rx_message):EncoderProcess(&CM3Encoder ,&rx_message);
									break;
						case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:   Moto_Flag_ID = 4;
									(can_count<=50) ? GetEncoderBias(&CM4Encoder ,&rx_message):EncoderProcess(&CM4Encoder ,&rx_message);
									break;
					}
					
			if(can_count >= 10000)
				can_count = 10000;
    }
}
