/*
 * MIT License
 * Copyright (c) 2021 _VIFEXTech
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __APP_H
#define __APP_H

#define ACCOUNT_SEND_NOTIFY_CMD(ACT, CMD)\
do{\
    AccountSystem::ACT##_Info_t info;\
    memset(&info, 0, sizeof(info));\
    info.cmd = AccountSystem::CMD;\
    AccountSystem::Broker()->AccountMaster.Notify(#ACT, &info, sizeof(info));\
}while(0)

#ifdef ARDUINO
#include "Port/Display.h"
#define INIT_DONE() \
do{                 \
    xTaskNotifyGive(handleTaskLvgl); \
}while(0) \

#else
#define INIT_DONE() \
do{                 \
}while(0) \

#endif

void App_Init();
void App_UnInit();

#endif
