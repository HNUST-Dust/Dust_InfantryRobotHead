#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

extern float g_roll,g_pitch,g_yaw;//欧拉角
extern float g_pitch_vision;
extern float g_yaw_vision;
extern float g_pitch_rad, g_yaw_rad;//弧度制欧拉角
extern float g_total_yaw;
extern float g_q[4];//四元数
extern float g_q_vision[4];//给视觉的四元数
extern float g_yaw_omega; //yaw角速度
extern float g_pitch_omega; //pitch角速度

void IMU_task();
void INS_Init(void);
#endif // IMU_TEMP_CTRL_H
