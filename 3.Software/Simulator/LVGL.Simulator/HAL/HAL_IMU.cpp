#include "HAL.h"
#include <stdlib.h>
#include "Accounts/Account_Master.h"

void HAL::IMU_Update()
{
    IMU_Info_t imu;
    imu.ax = rand() % 1000 - 500;
    imu.ay = rand() % 1000 - 500;
    imu.az = rand() % 1000 - 500;
    imu.gx = rand() % 1000 - 500;
    imu.gy = rand() % 1000 - 500;
    imu.gz = rand() % 1000 - 500;
    imu.roll = rand() % 1000 - 500;
    imu.yaw = rand() % 1000 - 500;
    imu.pitch = rand() % 1000 - 500;
    AccountSystem::IMU_Commit(&imu);
}
