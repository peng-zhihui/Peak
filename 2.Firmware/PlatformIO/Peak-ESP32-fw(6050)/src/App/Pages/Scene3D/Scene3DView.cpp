#include "Scene3DView.h"

using namespace Page;

void Scene3DView::Create(lv_obj_t* root)
{
	ui.group = lv_group_create();
	lv_indev_set_group(lv_get_indev(LV_INDEV_TYPE_ENCODER), ui.group);


	lv_obj_t* bgk = lv_obj_create(root);
	lv_obj_set_style_bg_opa(bgk, LV_OPA_100, LV_PART_MAIN);
	lv_obj_set_style_bg_color(bgk, lv_color_make(0, 0, 0), LV_PART_MAIN);
	lv_obj_set_size(bgk, 240, 240);//…Ë÷√∏≤∏«¥Û–° 


	lv_draw_rect_dsc_t rect_dsc;
	lv_draw_rect_dsc_init(&rect_dsc);
	rect_dsc.radius = 0;
	rect_dsc.bg_opa = LV_OPA_COVER;
	rect_dsc.bg_grad_dir = LV_GRAD_DIR_HOR;
	rect_dsc.bg_color = lv_color_make(0, 0, 0);
	rect_dsc.bg_grad_color = lv_color_make(0, 0, 0);
	rect_dsc.border_width = 1;
	rect_dsc.border_opa = LV_OPA_90;
	rect_dsc.border_color = lv_color_white();
	rect_dsc.shadow_width = 5;
	rect_dsc.shadow_ofs_x = 5;
	rect_dsc.shadow_ofs_y = 5;

	lv_draw_label_dsc_t label_dsc;
	lv_draw_label_dsc_init(&label_dsc);
	label_dsc.color = lv_color_make(0, 150, 150);

	static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(120, 120)];

	lv_obj_t* canvas = lv_canvas_create(root);
	lv_canvas_set_buffer(canvas, cbuf, 120, 120, LV_IMG_CF_TRUE_COLOR);
	lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0);
	lv_canvas_fill_bg(canvas, lv_color_make(0, 0, 0), LV_OPA_COVER);
	lv_canvas_draw_rect(canvas, 0, 0, 120, 120, &rect_dsc);
	lv_canvas_draw_text(canvas, 40, 20, 100, &label_dsc, "Some text on text canvas");


	lv_obj_t* label = lv_label_create(root);
	lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(label, lv_color_white(), 0);
	lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
	lv_label_set_text(label, "Interacting...");

	ui.labelTitle = label;
	ui.canvas = canvas;

	lv_group_add_obj(ui.group, ui.canvas);
	lv_group_focus_obj(ui.canvas);
}
