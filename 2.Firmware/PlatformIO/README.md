由于PlatformIO 的ESP32包更新太快新版编译又会报错，所以指定工程版本3.5.0（platform = espressif32@~3.5.0）

Peak-ESP32-fw(6050)      修改9250程序为6050

~~上一个版本在最新的platformio框架会报FreeRTOS路径错误和sd库中定义错误~~

~~由于platformio框架版本问题会报的错误解决方法：~~

~~HAL.h 文件  \#include "FreeRTOS.h"改为 \#include "FreeRTOS/FreeRTOS.h"~~

~~lib/SD/src/sd_diskio.cpp   添加头文件 \#include "diskio_impl.h"~~