#include "HAL/HAL.h"
#include "MPU6050.h"
#include "App/Accounts/Account_Master.h"

static MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t roll, yaw, pitch;

void HAL::IMU_Init()
{
    mpu.initialize();
    if (mpu.testConnection())
    {
        Serial.println("MPU connection failed.");
    }
}

void HAL::IMU_Update()
{
    IMU_Info_t imuInfo;
   // mpu.update();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu.getRotation(&roll,&yaw,&pitch);
    imuInfo.ax = ax;
    imuInfo.ay = ay;
    imuInfo.az = az;
    imuInfo.gx = gx;
    imuInfo.gy = gy;
    imuInfo.gz = gz;
    imuInfo.mx = 0; //6050无
    imuInfo.my = 0;
    imuInfo.mz = 0;
    imuInfo.roll = roll;
    imuInfo.yaw = yaw;
    imuInfo.pitch = pitch;

    AccountSystem::IMU_Commit(&imuInfo);
}
