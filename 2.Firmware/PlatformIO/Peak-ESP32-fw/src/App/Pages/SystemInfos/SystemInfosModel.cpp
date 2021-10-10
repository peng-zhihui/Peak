#include "SystemInfosModel.h"
#include <stdio.h>

using namespace Page;

void SystemInfosModel::Init()
{
    account = new Account("SystemInfosModel", AccountSystem::Broker(), 0, this);

    account->Subscribe("IMU");
    account->Subscribe("Power");
    account->Subscribe("Storage");
}

void SystemInfosModel::Deinit()
{
    if (account)
    {
        delete account;
        account = nullptr;
    }
}


void SystemInfosModel::GetIMUInfo(
    char* info, uint32_t len
)
{
    HAL::IMU_Info_t imu;
    account->Pull("IMU", &imu, sizeof(imu));
    snprintf(
        info,
        len,
        "%.3f\n%.3f\n%.3f\n%.3f\n%.3f\n%.3f\n%.3f\n%.3f\n%.3f",
        imu.ax,
        imu.ay,
        imu.az,
        imu.gx,
        imu.gy,
        imu.gz,
        imu.mx,
        imu.my,
        imu.mz
    );
}


void SystemInfosModel::GetBatteryInfo(
    int* usage,
    float* voltage,
    char* state, uint32_t len
)
{
    HAL::Power_Info_t power;
    account->Pull("Power", &power, sizeof(power));
    *usage = power.usage;
    *voltage = power.voltage / 1000.0f;
    strncpy(state, power.isCharging ? "CHARGE" : "DISCHARGE", len);
}

void SystemInfosModel::GetStorageInfo(
    bool* detect,
    char* usage, uint32_t len
)
{
    AccountSystem::Storage_Basic_Info_t info;
    account->Pull("Storage", &info, sizeof(info));
    *detect = info.isDetect;
    snprintf(
        usage, len,
        "%0.1f GB",
        info.totalSizeMB / 1024.0f
    );
}

void Page::SystemInfosModel::GetJointsInfo(char* data, uint32_t len)
{
    snprintf(
        data, len,
        "0\n0\n90\n0\n0\n0\n"
    );
}

void Page::SystemInfosModel::GetPose6DInfo(char* data, uint32_t len)
{
    snprintf(
        data, len,
        "222\n0\n307\n0\n90\n0\n"
    );
}
