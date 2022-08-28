#include "SystemInfos.h"
#include "App/Configs/Version.h"

using namespace Page;

SystemInfos::SystemInfos()
{
}

SystemInfos::~SystemInfos()
{

}

void SystemInfos::onCustomAttrConfig()
{

}

void SystemInfos::onViewLoad()
{
	Model.Init();
	View.Create(root);
	AttachEvent(root);
	AttachEvent(View.ui.joints.icon);
	AttachEvent(View.ui.pose6d.icon);
	AttachEvent(View.ui.system.icon);
	AttachEvent(View.ui.imu.icon);
	AttachEvent(View.ui.battery.icon);
	AttachEvent(View.ui.storage.icon);
}

void SystemInfos::onViewDidLoad()
{

}

void SystemInfos::onViewWillAppear()
{
	lv_indev_set_group(lv_get_indev(LV_INDEV_TYPE_ENCODER), View.ui.group);
	StatusBar::SetStyle(StatusBar::STYLE_BLACK);

	timer = lv_timer_create(onTimerUpdate, 100, this);
	lv_timer_ready(timer);

	View.SetScrollToY(root, -LV_VER_RES, LV_ANIM_OFF);
	lv_obj_fade_in(root, 300, 0);
}

void SystemInfos::onViewDidAppear()
{
	View.onFocus(View.ui.group);
}

void SystemInfos::onViewWillDisappear()
{
	lv_obj_fade_out(root, 300, 0);
}

void SystemInfos::onViewDidDisappear()
{
	lv_timer_del(timer);
}

void SystemInfos::onViewDidUnload()
{
	View.Delete();
	Model.Deinit();
}

void SystemInfos::AttachEvent(lv_obj_t* obj)
{
	lv_obj_set_user_data(obj, this);
	lv_obj_add_event_cb(obj, onEvent, LV_EVENT_PRESSED, this);
}

void SystemInfos::Update()
{
	char buf[64];

	/* Joints */
	Model.GetJointsInfo(buf, sizeof(buf));
	View.SetJoints(buf);

	/* Pose6D */
	Model.GetPose6DInfo(buf, sizeof(buf));
	View.SetPose6D(buf);

	/* IMU */
	Model.GetIMUInfo(buf, sizeof(buf));
	View.SetIMU(buf);

	/* Power */
	int usage;
	float voltage;
	Model.GetBatteryInfo(&usage, &voltage, buf, sizeof(buf));
	View.SetBattery(usage, voltage, buf);

	/* Storage */
	bool detect;
	Model.GetStorageInfo(&detect, buf, sizeof(buf));
	View.SetStorage(
		detect ? "YES" : "NO",
		buf,
		VERSION_FILESYSTEM
	);

	/* System */
	View.SetSystem(
		VERSION_FIRMWARE_NAME " " VERSION_SOFTWARE,
		VERSION_AUTHOR_NAME,
		VERSION_LVGL,
		"dummy time",
		VERSION_COMPILER,
		VERSION_BUILD_TIME
	);
}

void SystemInfos::onTimerUpdate(lv_timer_t* timer)
{
	SystemInfos* instance = (SystemInfos*)timer->user_data;

	instance->Update();
}

void SystemInfos::onEvent(lv_event_t* event)
{
	lv_obj_t* obj = lv_event_get_target(event);
	lv_event_code_t code = lv_event_get_code(event);
	auto* instance = (SystemInfos*)lv_obj_get_user_data(obj);

	if (code == LV_EVENT_PRESSED)
	{
		if (lv_obj_has_state(obj, LV_STATE_FOCUSED))
		{
			instance->Manager->Push("Pages/Scene3D");
		}
	}

 
}
