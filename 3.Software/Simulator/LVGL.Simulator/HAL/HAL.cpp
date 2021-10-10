#include "HAL/HAL.h"
#include "App/Configs/Version.h"

void HAL::HAL_Init()
{
    Buzz_init();
    Audio_Init();
}

void HAL::HAL_Update()
{
    IMU_Update();
    Audio_Update();
    Encoder_Update();
}
