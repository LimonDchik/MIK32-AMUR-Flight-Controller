#ifndef MIK32_MPU6050
#define MIK32_MPU6050

void gyro_init(void);
void gyro_processing(int* gyro_user);
void gyro_calibration(void);

#endif //MIK32_MPU6050