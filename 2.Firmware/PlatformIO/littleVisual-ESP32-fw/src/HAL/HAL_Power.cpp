#include "HAL/HAL.h"

/*上一次操作时间(ms)*/
static uint32_t Power_LastHandleTime = 0;

/*自动关机时间(秒)*/
static uint16_t Power_AutoLowPowerTimeout = 60;

/*自动关机功能使能*/
static bool Power_AutoLowPowerEnable = false;

static bool Power_IsShutdown = false;

static volatile uint16_t Power_ADCValue = 0;
static uint16_t Power_ADCValue_last[10] = {4095};

static HAL::Power_CallbackFunction_t Power_EventCallback = NULL;

#define BATT_MAX_VOLTAGE    3900
#define BATT_MIN_VOLTAGE    3300


static void Power_ADC_Init()
{
    pinMode(CONFIG_BAT_DET_PIN, INPUT);
    pinMode(CONFIG_BAT_CHG_DET_PIN, INPUT_PULLUP);
}

/**
  * @brief  电源初始化
  * @param  无
  * @retval 无
  */
void HAL::Power_Init()
{
    pinMode(CONFIG_BAT_CHG_DET_PIN, INPUT);

    /*电源使能保持*/
    Serial.println("Power: Waiting...");
    pinMode(CONFIG_POWER_EN_PIN, OUTPUT);
    digitalWrite(CONFIG_POWER_EN_PIN, LOW);
    // Try to connect to BLE device while waiting for boot
    uint64_t time = millis();
    while (millis() - time < 1000)
    {
        HAL::BT_Update();
        delay(100);
    }
    digitalWrite(CONFIG_POWER_EN_PIN, HIGH);
    Serial.println("Power: ON");

    /*电池检测*/
    Power_ADC_Init();
    Power_SetAutoLowPowerTimeout(60);
    Power_HandleTimeUpdate();
    Power_SetAutoLowPowerEnable(false);
}

static void Power_ADC_TrigUpdate()
{
    Power_ADCValue = analogRead(CONFIG_BAT_DET_PIN);
}

/**
  * @brief  更新操作时间
  * @param  无
  * @retval 无
  */
void HAL::Power_HandleTimeUpdate()
{
    Power_LastHandleTime = millis();
}

/**
  * @brief  设置自动关机时间
  * @param  sec:时间(秒)
  * @retval 无
  */
void HAL::Power_SetAutoLowPowerTimeout(uint16_t sec)
{
    Power_AutoLowPowerTimeout = sec;
}

/**
  * @brief  获取自动关机时间
  * @param  无
  * @retval sec:时间(秒)
  */
uint16_t HAL::Power_GetAutoLowPowerTimeout()
{
    return Power_AutoLowPowerTimeout;
}

/**
  * @brief  设置自动关机功能使能
  * @param  en:使能
  * @retval 无
  */
void HAL::Power_SetAutoLowPowerEnable(bool en)
{
    Power_AutoLowPowerEnable = en;
    Power_HandleTimeUpdate();
}

/**
  * @brief  执行关机
  * @param  无
  * @retval 无
  */
void HAL::Power_Shutdown()
{
    Backlight_SetGradual(0, 500);
    digitalWrite(CONFIG_POWER_EN_PIN, LOW);
    Power_IsShutdown = true;
}

/**
  * @brief  自动关机监控
  * @param  无
  * @retval 无
  */
void HAL::Power_Update()
{
    __IntervalExecute(Power_ADC_TrigUpdate(), 1000);

    if (!Power_AutoLowPowerEnable)
        return;

    if (Power_AutoLowPowerTimeout == 0)
        return;

    if (millis() - Power_LastHandleTime >= (Power_AutoLowPowerTimeout * 1000))
    {
        Power_Shutdown();
    }
}

void HAL::Power_GetInfo(Power_Info_t* info)
{
    uint32_t sum = Power_ADCValue;
    for (int i = 9; i > 0; i--)
    {
        Power_ADCValue_last[i] = Power_ADCValue_last[i - 1];
        sum += Power_ADCValue_last[i - 1];
    }
    Power_ADCValue_last[0] = Power_ADCValue;

    int voltage = map(
        sum / 10,
        0, 4095,
        0, 3300
    );

    voltage *= 2;

    __LimitValue(voltage, BATT_MIN_VOLTAGE, BATT_MAX_VOLTAGE);

    int usage = map(
        voltage,
        BATT_MIN_VOLTAGE, BATT_MAX_VOLTAGE,
        0, 100
    );

    info->usage = usage;
    info->isCharging = usage != 100 && !digitalRead(CONFIG_BAT_CHG_DET_PIN);
    info->voltage = voltage;
}
