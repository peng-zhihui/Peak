#include "Account_Master.h"
#include "HAL/HAL.h"

static AccountBroker dataCenter("MASTER");

AccountBroker* AccountSystem::Broker()
{
    return &dataCenter;
}

void Accounts_Init()
{
#define ACT_DEF(NodeName, bufferSize)\
    Account* act##NodeName = new Account(#NodeName, &dataCenter, bufferSize);

#include "_ACT_LIST.inc"

#undef ACT_DEF

#define ACT_DEF(NodeName, bufferSize)\
do{\
    extern void _ACT_##NodeName##_Init(Account* account);\
    _ACT_##NodeName##_Init(act##NodeName);\
}while(0)

#include "_ACT_LIST.inc"

#undef ACT_DEF
}
