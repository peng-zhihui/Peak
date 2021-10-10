#ifndef __DATA_PROC_H
#define __DATA_PROC_H

#include "App/Utils/AccountSystem/AccountBroker.h"
#include "HAL/HAL_Def.h"
#include "ACT_Def.h"

#define ACCOUNT_INIT_DEF(name) void _ACT_##name##_Init(Account* account)

void Accounts_Init();

namespace AccountSystem
{

AccountBroker* Broker();

void IMU_Commit(HAL::IMU_Info_t* info);
}

#endif
