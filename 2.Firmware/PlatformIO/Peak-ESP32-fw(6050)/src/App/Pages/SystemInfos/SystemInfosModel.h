#ifndef __SYSTEM_INFOS_MODEL_H
#define __SYSTEM_INFOS_MODEL_H

#include "App/Accounts/Account_Master.h"

namespace Page
{

class SystemInfosModel
{
public:
    void Init();
    void Deinit();

    void GetIMUInfo(
        char* info, uint32_t len
    );

    void GetBatteryInfo(
        int* usage,
        float* voltage,
        char* state, uint32_t len
    );

    void GetStorageInfo(
        bool* detect,
        char* size, uint32_t len
    );

    void GetJointsInfo(
        char* data, uint32_t len
    );

    void GetPose6DInfo(
        char* data, uint32_t len
    );

private:
    Account* account;

private:

};

}

#endif
