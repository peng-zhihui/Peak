#include "HAL/HAL.h"
#include "MPU9250.h"
#include "App/Accounts/Account_Master.h"

static MPU9250 mpu;

void HAL::IMU_Init()
{
    if (!mpu.setup(0x68))
    {
        Serial.println("MPU connection failed.");
    }
}

void HAL::IMU_Update()
{
    IMU_Info_t imuInfo;
    mpu.update();

    imuInfo.ax = mpu.getAccX();
    imuInfo.ay = mpu.getAccY();
    imuInfo.az = mpu.getAccZ();
    imuInfo.gx = mpu.getGyroX();
    imuInfo.gy = mpu.getGyroY();
    imuInfo.gz = mpu.getGyroZ();
    imuInfo.mx = mpu.getMagX();
    imuInfo.my = mpu.getMagY();
    imuInfo.mz = mpu.getMagZ();
    imuInfo.roll = mpu.getRoll();
    imuInfo.yaw = mpu.getYaw();
    imuInfo.pitch = mpu.getPitch();

    AccountSystem::IMU_Commit(&imuInfo);
}
