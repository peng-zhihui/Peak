#ifndef __STARTUP_MODEL_H
#define __STARTUP_MODEL_H

#include "App/Accounts/Account_Master.h"
#include "HAL/HAL.h"

namespace Page
{

class StartupModel
{
public:
    void Init();
    void DeInit();
    void PlayMusic(const char* music);
    void SetEncoderEnable(bool en)
    {
        HAL::Encoder_SetEnable(en);
    }

private:
    Account* account;
};

}

#endif
