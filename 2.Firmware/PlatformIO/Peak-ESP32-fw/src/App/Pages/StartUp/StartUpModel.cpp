#include "StartupModel.h"

using namespace Page;

void StartupModel::Init()
{
    account = new Account("StartupModel", AccountSystem::Broker(), 0, this);
}

void StartupModel::DeInit()
{
    if (account)
    {
        delete account;
        account = nullptr;
    }
}

void StartupModel::PlayMusic(const char* music)
{
    AccountSystem::MusicPlayer_Info_t info;
    info.music = music;
    account->Notify("MusicPlayer", &info, sizeof(info));
}
