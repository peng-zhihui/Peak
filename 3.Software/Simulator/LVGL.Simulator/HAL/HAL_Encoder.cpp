#include "HAL.h"
#include "App/Accounts/Account_Master.h"

Account actEncoder("Encoder", AccountSystem::Broker(), sizeof(int), nullptr);

void HAL::Encoder_Init()
{

}

void HAL::Encoder_Update()
{
    static int val = 0;
    actEncoder.Commit(&val, sizeof(int));
    actEncoder.Publish();
}

int32_t HAL::Encoder_GetDiff()
{
    return 0;
}

bool HAL::Encoder_GetIsPush()
{
    return false;
}

void HAL::Encoder_SetEnable(bool en)
{
}
