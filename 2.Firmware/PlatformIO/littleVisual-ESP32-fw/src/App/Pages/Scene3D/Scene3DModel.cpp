#include "Scene3DModel.h"
#include "App/Accounts/Account_Master.h"

using namespace Page;


#define SCREENW         120
#define SCREENH         120
#define HALFW            60
#define HALFH            60
#define FOV              64
#define SKIP_TICKS       20.0 // 50fps
#define MAX_FRAMESKIP     5
#define COLOR0 lv_color_make(255, 255, 255)
#define COLOR1 lv_color_make(0, 0, 0)
#define COLOR2 ST7735_GREEN
#define COLOR3 ST7735_YELLOW
#define COLOR4 ST7735_BLUE
#define COLOR5 ST7735_CYAN
#define COLOR6 ST7735_RED
#define COLOR7 ST7735_MAGENTA
#define LUT(a) (long)( lut[a]) // return value from LUT

float scale = 1.8;

const unsigned int lut[] = {         // 0 to 90 degrees fixed point COSINE look up table
    16384, 16381, 16374, 16361, 16344, 16321, 16294, 16261, 16224, 16182, 16135, 16082, 16025, 15964,
    15897, 15825, 15749, 15668, 15582, 15491, 15395, 15295, 15190, 15081, 14967, 14848, 14725, 14598,
    14466, 14329, 14188, 14043, 13894, 13740, 13582, 13420, 13254, 13084, 12910, 12732, 12550, 12365,
    12175, 11982, 11785, 11585, 11381, 11173, 10963, 10748, 10531, 10310, 10086, 9860, 9630, 9397,
    9161, 8923, 8682, 8438, 8191, 7943, 7691, 7438, 7182, 6924, 6663, 6401, 6137, 5871, 5603, 5334,
    5062, 4790, 4516, 4240, 3963, 3685, 3406, 3126, 2845, 2563, 2280, 1996, 1712, 1427, 1142, 857,
    571, 285, 0
};

long SIN(unsigned int angle)
{
    angle += 90;
    if (angle > 450) return LUT(0);
    if (angle > 360 && angle < 451) return -LUT(angle - 360);
    if (angle > 270 && angle < 361) return -LUT(360 - angle);
    if (angle > 180 && angle < 271) return LUT(angle - 180);
    return LUT(180 - angle);
}

long COS(unsigned int angle)
{
    if (angle > 360) return LUT(0);
    if (angle > 270 && angle < 361) return LUT(360 - angle);
    if (angle > 180 && angle < 271) return -LUT(angle - 180);
    if (angle > 90 && angle < 181) return -LUT(180 - angle);
    return LUT(angle);
}

// ----------------------------------------------
// Matrix operation
// ----------------------------------------------
Matrix4 mMultiply(const Matrix4 &mat1, const Matrix4 &mat2)
{
    Matrix4 mat;
    unsigned char r, c;
    for (c = 0; c < 4; c++)
        for (r = 0; r < 4; r++)
            mat.m[c][r] = pMultiply(mat1.m[0][r], mat2.m[c][0]) +
                          pMultiply(mat1.m[1][r], mat2.m[c][1]) +
                          pMultiply(mat1.m[2][r], mat2.m[c][2]) +
                          pMultiply(mat1.m[3][r], mat2.m[c][3]);
    return mat;
}

Matrix4 mRotateX(const unsigned int angle)
{
    Matrix4 mat;
    mat.m[1][1] = COS(angle);
    mat.m[1][2] = SIN(angle);
    mat.m[2][1] = -SIN(angle);
    mat.m[2][2] = COS(angle);
    return mat;
}

Matrix4 mRotateY(const unsigned int angle)
{
    Matrix4 mat;
    mat.m[0][0] = COS(angle);
    mat.m[0][2] = -SIN(angle);
    mat.m[2][0] = SIN(angle);
    mat.m[2][2] = COS(angle);
    return mat;
}

Matrix4 mRotateZ(const unsigned int angle)
{
    Matrix4 mat;
    mat.m[0][0] = COS(angle);
    mat.m[0][1] = SIN(angle);
    mat.m[1][0] = -SIN(angle);
    mat.m[1][1] = COS(angle);
    return mat;
}

Matrix4 mTranslate(const long x, const long y, const long z)
{
    Matrix4 mat;
    mat.m[3][0] = x << PSHIFT;
    mat.m[3][1] = y << PSHIFT;
    mat.m[3][2] = z << PSHIFT;
    return mat;
}

Matrix4 mScale(const float ratio)
{
    Matrix4 mat;
    mat.m[0][0] *= ratio;
    mat.m[1][1] *= ratio;
    mat.m[2][2] *= ratio;
    return mat;
}

// ----------------------------------------------
// Shoelace algorithm to get the surface
// ----------------------------------------------
int shoelace(const int(* n)[2], const unsigned char index)
{
    unsigned char t = 0;
    int surface = 0;
    for (; t < 3; t++)
    {
        // (x1y2 - y1x2) + (x2y3 - y2x3) ...
        surface += (n[EDGE(index, t)][0] * n[EDGE(index, (t < 2 ? t + 1 : 0))][1]) -
                   (n[EDGE(index, (t < 2 ? t + 1 : 0))][0] * n[EDGE(index, t)][1]);
    }
    return surface * 0.5;
}

// ----------------------------------------------
// Shoelace algorithm for triangle visibility
// ----------------------------------------------
bool is_hidden(const int(* n)[2], const unsigned char index)
{
    // (x1y2 - y1x2) + (x2y3 - y2x3) ...
    return (((n[EDGE(index, 0)][0] * n[EDGE(index, 1)][1]) -
             (n[EDGE(index, 1)][0] * n[EDGE(index, 0)][1])) +
            ((n[EDGE(index, 1)][0] * n[EDGE(index, 2)][1]) -
             (n[EDGE(index, 2)][0] * n[EDGE(index, 1)][1])) +
            ((n[EDGE(index, 2)][0] * n[EDGE(index, 0)][1]) -
             (n[EDGE(index, 0)][0] * n[EDGE(index, 2)][1]))) >= 0;
}

// ----------------------------------------------
// draw projected nodes
// ----------------------------------------------
void Scene3DModel::draw_vertex(const uint16_t color, lv_obj_t* canvas)
{
    int i = NODECOUNT - 1;
    lv_canvas_fill_bg(canvas, COLOR1, LV_OPA_COVER);
    do
    {
        lv_canvas_set_px(canvas, proj_nodes[i][0], proj_nodes[i][1], COLOR0);
    } while (i--);
}

// ----------------------------------------------
// draw edges between projected nodes
// ----------------------------------------------
void Scene3DModel::draw_wireframe(const uint16_t color, lv_obj_t* canvas)
{
    int i = TRICOUNT - 1;
    lv_canvas_fill_bg(canvas, COLOR1, LV_OPA_COVER);

    do
    {
        // don't draw triangle with negative surface value
        if (!is_hidden(proj_nodes, i))
        {
            lv_draw_line_dsc_t line_dsc;
            lv_draw_line_dsc_init(&line_dsc);
            line_dsc.color = COLOR0;
            line_dsc.width = 1;

            lv_point_t line_point[3];
            line_point[0].x = proj_nodes[EDGE(i, 0)][0];
            line_point[0].y = proj_nodes[EDGE(i, 0)][1];
            line_point[1].x = proj_nodes[EDGE(i, 1)][0];
            line_point[1].y = proj_nodes[EDGE(i, 1)][1];
            line_point[2].x = proj_nodes[EDGE(i, 2)][0];
            line_point[2].y = proj_nodes[EDGE(i, 2)][1];
            lv_canvas_draw_line(canvas, line_point, 3, &line_dsc);
        }
    } while (i--);
}

// ----------------------------------------------
// draw flat color (not flat shading)
// ----------------------------------------------
void Scene3DModel::draw_flat_color(const int(* n)[2], uint16_t color, lv_obj_t* canvas)
{
    int i = TRICOUNT - 1;
    int surface;
    uint16_t col = color;
    do
    {
        // draw only triangles facing us
        if ((surface = shoelace(n, i)) < 0)
        {
            // this is an ugly hack but it 'somehow' fakes shading
            // depending on the size of the surface of the triangle
            // change the color toward brighter/darker
            color = col * (surface * 0.001);

            //            TFT.fillTriangle(n[EDGE(i, 0)][0], n[EDGE(i, 0)][1],
            //                             n[EDGE(i, 1)][0], n[EDGE(i, 1)][1],
            //                             n[EDGE(i, 2)][0], n[EDGE(i, 2)][1],
            //                             color);
        }
    } while (i--);
}

// ----------------------------------------------
// clear frame using bounding box (dirty mask)
// ----------------------------------------------
void Scene3DModel::clear_dirty(const int(* n)[2], lv_obj_t* canvas)
{
    unsigned char x0 = SCREENW, y0 = SCREENH, x1 = 0, y1 = 0, c, w, h;
    // get bounding box of mesh
    for (c = 0; c < NODECOUNT; c++)
    {
        if (n[c][0] < x0) x0 = n[c][0];
        if (n[c][0] > x1) x1 = n[c][0];
        if (n[c][1] < y0) y0 = n[c][1];
        if (n[c][1] > y1) y1 = n[c][1];
    }
    //    // clear area
    //    TFT.spi_begin();
    //    TFT.setAddrWindow_(x0, y0, x1, y1);
    //    h = (y1 - y0);
    //    w = (x1 - x0) + 1;
    //    do
    //    {
    //        TFT.spiWrite16(COLOR0, w);
    //    } while (h--);
    //    TFT.spi_end();
}
void Scene3DModel::Update(lv_obj_t* canvas)
{
    // rotation
    m_world = mRotateX(mesh_rotation.x);
    m_world = mMultiply(mRotateY(mesh_rotation.y), m_world);
    m_world = mMultiply(mRotateZ(mesh_rotation.z), m_world);
    // scaling
    m_world = mMultiply(mScale(scale), m_world);

    // project nodes with world matrix
    Vector3i p;
    for (int i = 0; i < NODECOUNT; i++)
    {
        p.x = (m_world.m[0][0] * (NODE(i, 0) >> PSHIFT) +
               m_world.m[1][0] * (NODE(i, 1) >> PSHIFT) +
               m_world.m[2][0] * (NODE(i, 2) >> PSHIFT) +
               m_world.m[3][0]) / PRES;

        p.y = (m_world.m[0][1] * (NODE(i, 0) >> PSHIFT) +
               m_world.m[1][1] * (NODE(i, 1) >> PSHIFT) +
               m_world.m[2][1] * (NODE(i, 2) >> PSHIFT) +
               m_world.m[3][1]) / PRES;

        p.z = (m_world.m[0][2] * (NODE(i, 0) >> PSHIFT) +
               m_world.m[1][2] * (NODE(i, 1) >> PSHIFT) +
               m_world.m[2][2] * (NODE(i, 2) >> PSHIFT) +
               m_world.m[3][2]) / PRES;

        // store projected node
        proj_nodes[i][0] = (FOV * p.x) / (FOV + p.z) + HALFW;
        proj_nodes[i][1] = (FOV * p.y) / (FOV + p.z) + HALFH;
    }


    // default auto-rotation mode
    mesh_rotation.x += 2;
    mesh_rotation.y += 2;
    mesh_rotation.z++;

    if (mesh_rotation.x > 360) mesh_rotation.x = 0;
    if (mesh_rotation.y > 360) mesh_rotation.y = 0;
    if (mesh_rotation.z > 360) mesh_rotation.z = 0;

    //draw_vertex(0, canvas);
    draw_wireframe(0, canvas);
}

static int onEvent(Account* account, Account::EventParam_t* param)
{
    scale += (float) (*((int16_t*) (param->data_p))) * 0.05f;

    return 0;
}


void Scene3DModel::Init()
{
    account = new Account("Scene3DModel", AccountSystem::Broker(), 0, this);

    account->SetEventCallback(onEvent);
    account->Subscribe("Encoder");
}
