#include "control.h"	
#include "filter.h"	
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm,count,fflag;
u8 Flag_Target;
int TIM1_UP_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms定时中断
	{   
		 TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 
		  Flag_Target=!Flag_Target;
		  if(Flag_Target==1)                                       //===5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			{
			Get_Angle(Way_Angle);                                    //===更新姿态	
			return 0;	
			}                                                        //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
	  	Get_Angle(Way_Angle);                                    //===更新姿态	
  		Led_Flash(100);                                          //===LED闪烁;指示单片机正常运行	   每秒反转一次灯的状态
			Key();                                                   //===扫描按键状态
			
			if(++count>20)  fflag=!fflag,count=0;
				
			if(fflag==1)
			{
			  PAout(0)=1;
				PAout(1)=1;
				PAout(2)=1;
				PAout(3)=1;
				PAout(13)=1;
				PAout(14)=1;
				
				PBout(4)=1;
				PBout(5)=1;
				PBout(6)=1;
				PBout(7)=1;
				PBout(8)=1;
				PBout(9)=1;
			}	
				else
			{
			  PAout(0)=0;
				PAout(1)=0;
				PAout(2)=0;
				PAout(3)=0;
				PAout(13)=0;
				PAout(14)=0;
				
				PBout(4)=0;
				PBout(5)=0;
				PBout(6)=0;
				PBout(7)=0;
				PBout(8)=0;
				PBout(9)=0;
			}	
			
 		
	}       	
	 return 0;	  
} 
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
			if(moto1>0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto1);
		  if(moto2>0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}

/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click(); 
	if(tmp==1)Flag_Show=!Flag_Show;
//	if(tmp==2)Flag_Show=~Flag_Show;
}


/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 1：无  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	    if(way==1)                                      //DMP没有涉及到严格的时序问题，在主函数读取
			{	
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度记
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度记
		  if(Gyro_Y>32768)  Gyro_Y-=65536;     //数据类型转换  也可通过short强制类型转换
			if(Gyro_Z>32768)  Gyro_Z-=65536;     //数据类型转换
	  	if(Accel_X>32768) Accel_X-=65536;    //数据类型转换
		  if(Accel_Z>32768) Accel_Z-=65536;    //数据类型转换
			Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //计算与地面的夹角	
		  Gyro_Y=Gyro_Y/16.4;                                    //陀螺仪量程转换	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//卡尔曼滤波	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //互补滤波
	    Angle_Balance=angle;                                   //更新平衡倾角
	  	}
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
