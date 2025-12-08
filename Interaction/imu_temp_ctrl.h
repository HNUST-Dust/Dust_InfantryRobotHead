#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

extern float g_roll,g_pitch,g_yaw;//欧拉角
extern float g_q[4];//四元数

void IMU_task();
void INS_Init(void);
#endif // IMU_TEMP_CTRL_H
