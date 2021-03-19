#include "icm20602.h"

#define delayms(x) HAL_Delay(x)

void ICM_Write_Byte(uint8_t reg,uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;          //�ȷ��ͼĴ���
	buff[1] = value;        //�ٷ�������
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, buff, 2, 0xffff);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}

uint8_t ICM_Read_Byte(uint8_t reg)
{
	uint8_t buff[2];
	uint8_t rece[2];
  buff[0] = reg | 0x80;//�ȷ��ͼĴ���
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, buff, rece, 2, 0xffff);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
  return rece[1];
}

void ICM_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{   
	buf[0] = reg | 0x80;
	/* д��Ҫ���ļĴ�����ַ */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, buf, buf, len+1, 0xffff);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}

void ICM20602_Init(void)
{
	uint8_t res;
  delayms(100);
  res=ICM_Read_Byte(WHO_AM_I);//��ȡICM20602��ID
	if(res == ICM20602_ID)   
	{
		printf("icm20602 is ok\r\n");
	}
  res = 0;
  ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X80);//��λ
  delayms(100);  //��ʱ100ms
  ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X00);//����
  delayms(100);  //��ʱ100ms

  ICM_Set_Gyro_Fsr(3);			       
  ICM_Set_Accel_Fsr(0);	
  ICM_Set_Rate(1000);					   //���ò�����1000Hz
  ICM_Write_Byte(ICM_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
  ICM_Write_Byte(ICM_INT_EN_REG,0X00);   //�ر������ж�
  ICM_Write_Byte(ICM_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
  ICM_Write_Byte(ICM_PWR_MGMT1_REG,0X01);//����CLKSEL,PLL X��Ϊ�ο�
  ICM_Write_Byte(ICM_PWR_MGMT2_REG,0X00);//���ٶ��������Ƕ�����  
}

//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void ICM_Set_Gyro_Fsr(uint8_t fsr)
{
	ICM_Write_Byte(ICM_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void ICM_Set_Accel_Fsr(uint8_t fsr)
{
	ICM_Write_Byte(ICM_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void ICM_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	ICM_Write_Byte(ICM_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
void ICM_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	ICM_Write_Byte(ICM_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	ICM_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��Ӽ�ֵ�����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
void ICM_Get_Raw_data(int16_t *ax,int16_t *ay,int16_t*az,int16_t *gx,int16_t *gy,int16_t *gz)
{
	uint8_t buf[15];  
	ICM_Read_Len(ICM_ACCEL_XOUTH_REG,14,buf);
    
  *ax=((uint16_t)buf[1]<<8)|buf[2];  
  *ay=((uint16_t)buf[3]<<8)|buf[4];  
  *az=((uint16_t)buf[5]<<8)|buf[6];
  *gx=((uint16_t)buf[9]<<8)|buf[10];  
  *gy=((uint16_t)buf[11]<<8)|buf[12];  
  *gz=((uint16_t)buf[13]<<8)|buf[14];	
}

//��Ԫ����̬����
static float NormAccz;
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float Gyro_G = 0.03051756f;	
const float Gyro_Gr = 0.0005326f; 
#define squa( Sq )   (((float)Sq)*((float)Sq))
float Q_rsqrt(float number);

void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) 
{		
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};  // ��Ԫ��
	float q0_t,q1_t,q2_t,q3_t;
	float NormQuat; 
	float HalfTime = dt * 0.5f;  
	// ��ȡ��Ч��ת�����е��������� 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	
	// ���ٶȹ�һ��
  NormQuat = Q_rsqrt(squa(pMpu->accX)+ squa(pMpu->accY) +squa(pMpu->accZ));
	
  Acc.x = pMpu->accX * NormQuat;
  Acc.y = pMpu->accY * NormQuat;
  Acc.z = pMpu->accZ * NormQuat;	
	
 	//������˵ó���ֵ
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
	//�������ٶȻ��ֲ������ٶȵĲ���ֵ
  GyroIntegError.x += AccGravity.x * KiDef;
  GyroIntegError.y += AccGravity.y * KiDef;
  GyroIntegError.z += AccGravity.z * KiDef;
	
	//���ٶ��ںϼ��ٶȻ��ֲ���ֵ
  Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//������
  Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
  Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	
	// һ�����������, ������Ԫ��
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// ��Ԫ����һ��
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	
	// ��Ԫ��תŷ����
	{
		/*��������ϵ�µ�Z��������*/
		float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*����(3,1)��*/
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*����(3,2)��*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*����(3,3)��*/		 
#ifdef	YAW_GYRO
		*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
#else
		float yaw_G = pMpu->gyroZ * Gyro_G;
		if((yaw_G > 3.0f) || (yaw_G < -3.0f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
		{
			pAngE->yaw  += yaw_G * dt;			
		}
#endif
		pAngE->pitch  =  asin(vecxZ)* RtA;						
		
		pAngE->roll	= atan2f(vecyZ,veczZ) * RtA;	//PITCH 		
        
		NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;	/*Z����ٶ�*/				
	}
}

float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );
	return y;
} 
