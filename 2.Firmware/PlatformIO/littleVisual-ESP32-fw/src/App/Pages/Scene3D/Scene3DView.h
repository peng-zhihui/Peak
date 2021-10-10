#ifndef __SCENE3D_VIEW_H
#define __SCENE3D_VIEW_H

#include "App/Pages/Page.h"

namespace Page
{
    class Scene3DView
    {
    public:
        void Create(lv_obj_t* root);

    public:
        struct
        {
            lv_obj_t* labelTitle;
            lv_obj_t* canvas;
            lv_group_t* group;
        } ui;

    private:
    };
}

#endif // !__VIEW_H
