#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
// #include "adc.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "alg_math.h"
#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define temp_times 300       //探测温度阈值

pid_type_def Temperature_PID={0};
float Temperature_PID_Para[3] = {1600,50,40};
float H723_Temperature = 0.f;
unsigned int adc_v;
float adcx;

float gyro[3], accel[3], temp; //陀螺仪原始值
float gyro_correct[3]={0};  //0飘初始值
float RefTemp = 40;   //Destination
float g_roll,g_pitch,g_yaw = 0;//欧拉角
float g_pitch_vision;
float g_yaw_vision;
float g_yaw_omega; //yaw角速度
float g_pitch_rad, g_yaw_rad = 0;//弧度制欧拉角
float g_total_yaw=0;
float g_q[4] = {0,0,0,0};//四元数
float g_q_vision[4] = {0,0,0,0};//给视觉的四元数
uint8_t attitude_flag=1;
uint32_t correct_times=0;


void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
	PID_init(&Temperature_PID, PID_POSITION,Temperature_PID_Para,2000,200); //加热pidlimit
	Mahony_Init(1000);  //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init());  //陀螺仪初始化
}

uint32_t temp_temperature=0;
void IMU_Temperature_Ctrl(){
	  PID_calc(&Temperature_PID, temp, RefTemp); //温度pid  //需要调一下pid使得温度在40°左右
		temp_temperature=(uint32_t)Temperature_PID.out; 
		if(Temperature_PID.out<0)
		{
			temp_temperature=0;
		}
    htim3.Instance->CCR4 = temp_temperature;
}
/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
 
*/
static uint8_t first_mahony=0; 
void INS_Task(void)  //1khz
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(gyro, accel, &temp); //陀螺仪值读取
		if(first_mahony==0)
		{
			first_mahony++;
			MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);   //mahony上电快速初始化
		}
		if(attitude_flag==2)  //ekf的姿态解算
		{
			gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
			gyro[1]-=gyro_correct[1];
			gyro[2]-=gyro_correct[2];
			
			#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
				if(fabsf(gyro[2])<0.003f)
					gyro[2]=0;
			#endif
			//===========================================================================
				//ekf姿态解算部分
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
			IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			//===============================================================================	
				
			//=================================================================================
			//mahony姿态解算部分 
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
			Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
			Mahony_computeAngles(); //角度计算
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			//=============================================================================
			//ekf获取姿态角度函数
			g_pitch = Get_Pitch(); //获得pitch,视觉上位机要求改成反方向的
			g_pitch_vision = - g_pitch;
			g_yaw_vision = - g_yaw;
			g_roll = Get_Roll();//获得roll
			g_yaw = Get_Yaw();//获得yaw
			g_total_yaw=Get_YawTotal(); //获得总yaw角度
			g_yaw_omega = Get_YawOmega(); //获得yaw角速度
			g_pitch_rad = g_pitch * DEG_TO_RAD; //弧度制pitch,
			g_yaw_rad = g_yaw * DEG_TO_RAD;     //弧度制yaw
			Get_q(g_q); //获得四元数

			// 重新根据欧拉角重构四元数，但将pitch取反，使得通过四元数解算出的pitch方向相反，yaw/roll保持不变
		
			float phi = g_roll * DEG_TO_RAD;    // roll (rad)
			float theta = g_pitch_rad;          // pitch (rad)
			float psi = g_yaw_rad;              // yaw (rad)
			theta = -theta; // 取反pitch

			float cy = cosf(psi * 0.5f);
			float sy = sinf(psi * 0.5f);
			float cp = cosf(theta * 0.5f);
			float sp = sinf(theta * 0.5f);
			float cr = cosf(phi * 0.5f);
			float sr = sinf(phi * 0.5f);

			// 四元数 (w, x, y, z) 对应 ZYX (yaw-pitch-roll) 顺序构造
			float qw = cy * cp * cr + sy * sp * sr;
			float qx = cy * cp * sr - sy * sp * cr;
			float qy = cy * sp * cr + sy * cp * sr;
			float qz = sy * cp * cr - cy * sp * sr;

			g_q_vision[0] = qw;
			g_q_vision[1] = qx;
			g_q_vision[2] = qy;
			g_q_vision[3] = qz;
            
			//==============================================================================
		}
		else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
		{
			//gyro correct
			gyro_correct[0]+=	gyro[0];
			gyro_correct[1]+=	gyro[1];
			gyro_correct[2]+=	gyro[2];
			correct_times++;
			if(correct_times>=correct_Time_define)
			{
				gyro_correct[0]/=correct_Time_define;
				gyro_correct[1]/=correct_Time_define;
				gyro_correct[2]/=correct_Time_define;
				attitude_flag=2; // go to 2 state
			}
		}
    }

    // temperature control
    if ((count % 10) == 0)
    {
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl();
		//==================================================================================
		//单片机内部温度测量
				// HAL_ADC_Start(&hadc3);
				// adc_v = HAL_ADC_GetValue(&hadc3);
				// adcx = (110.0-30.0)/(*(unsigned short*)(0x1FF1E840) - *(unsigned short*)(0x1FF1E820));
				// H723_Temperature = adcx*(adc_v - *(unsigned short*)(0x1FF1E820))+30;
		//========================================================================================
			static uint32_t temp_Ticks=0;
			if((fabsf(temp-RefTemp)<0.5f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
			{
				temp_Ticks++;
				if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
				{
					attitude_flag=1;  //go to correct state
				}
			}
    }
    count++;
}

void IMU_task()
{

    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(pdMS_TO_TICKS(1));
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
