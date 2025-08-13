#include <algorithm>
#include <array>
#include <iostream>

#include "geometry.h"
#include "model.h"
#include "tgaimage.h"

// BGRA = blue green read alpha(不透明度)
const TGAColor white = TGAColor {255, 255, 255, 255};
const TGAColor blue = TGAColor {255, 0, 0, 255};
const TGAColor green = TGAColor {0, 255, 0, 255};
const TGAColor red = TGAColor {0, 0, 255, 255};

// 按步长 t 进行计算
// 如何选择步长是一个问题，太小，性能开销大，太大，映射到的像素变离散不连续了
void line1(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    for (float t = 0.; t < 1.; t += 0.01) {
        int x = x0 + (x1 - x0) * t;
        int y = y0 + (y1 - y0) * t;
        image.set(x, y, color);
    }
}

// 考虑到相邻像素 x 轴 y 轴最小移动单位为 1，这里选择按 x 轴步长为 1 计算映射的像素点
// 问题
// 1. 以 x 轴步长 1 计算，当斜率比较大时，例如dy/dx > 1，仍会出现映射像素点离散不连续的情况
// 2. 只考虑了 x0<x1 起点在左，终点在右的情况
void line2(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    for (int x = x0; x <= x1; x++) {
        float t = (x - x0) / static_cast<float>(x1 - x0);
        int y = y0 + (y1 - y0) * t;
        image.set(x, y, color);
    }
}

// 当斜率大时，换用 y 轴步长 1 进行计算
// 当 x0>x1 左终点右起点时，转换为左起点右终点进行计算
void line3(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    // 斜率高于1，使用斜率低的 y 作为自变量
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {  // if the line is steep, we transpose the image
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }

    // 起点在右边，替换起点终点
    if (x0 > x1) {  // make it left−to−right
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    for (int x = x0; x <= x1; x++) {
        float t = (x - x0) / static_cast<float>(x1 - x0);
        int y = y0 * (1.0 - t) + y1 * t;
        if (steep) {
            image.set(y, x, color);  // if transposed, de−transpose
        } else {
            image.set(x, y, color);
        }
    }
}

// 中点算法
// 进行性能优化
// 将 line3() 循环中的步长提取出来，省去每次循环计算斜率的除法运算
// 循环中也使用四舍五入的方式，省去乘法运算
// 由此，循环中只有加减运算
void line4(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {  // if the line is steep, we transpose the image
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {  // make it left−to−right
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    float k = std::abs(dy / static_cast<float>(dx));  // 斜率
    float total_dy = 0;                               // 用于记录累计的 y 增量

    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            image.set(y, x, color);  // if transposed, de−transpose
        } else {
            image.set(x, y, color);
        }
        total_dy += k;
        // 随 x 递增 1，y 每次增量为 斜率*步长 = k
        // 直到 y 随 x 的增量大于 0.5 时（斜率小时，可能多次循环才会大于 0.5），
        // 才 y+1 映射到更高一层的像素点（四舍五入映射到最近的像素点）
        if (total_dy > 0.5) {
            y += y1 > y0 ? 1 : -1;
            total_dy -= 1.0;
        }
    }
}

// Bresenhan 算法
// 更进一步的优化
// line4() 循环中 total_dy 是与 0.5 这个浮点数进行比较，仍然是浮点数运算
// 这一步是为了计算累计增量 total_dy 是否大于 0.5，
// 那么我们把两边都翻倍，2*total_dy 与 1 进行比较，此时 0.5->1 浮点数转变为了整型
// total_dy 来源于斜率 k * 步长，我们计算斜率 k 时，把斜率结果再乘以2 即可 k2 = 2k
// float k2 = 2 * std::abs(dy / static_cast<float>(dx));
// ...
//     total_dy += k2;
//     if (total_dy > 1) {
//         y += y1 > y0 ? 1 : -1;
//         total_dy -= 2.0;
//     }
// 到这里，total_dy 仍然是浮点数。注意到
// k2 = 2 * dy/dx
// total_dy = n * k2 > 1 (n 为循环次数)
// total_dy = n * 2 * dy/dx > 1
// 同样的，只需要把分母 dx 提出来，即 k3 = 2 * k * dx, 即可使得不等式变为 total_dy > dx;
// 这样，甚至斜率 k3, total_dy 也变为了可使用整型
// int k3 = 2 * std::abs(dy);
// ...
//     total_dy += k3;
//     if (total_dy > dx) {
//         y += y1 > y0 ? 1 : -1;
//         total_dy -= 2 * dx;
//     }
// 优化后整个 line5() 中只有整数运算
void line5(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {  // if the line is steep, we transpose the image
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {  // make it left−to−right
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    int k3 = 2 * std::abs(dy);
    int total_dy = 0;

    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            image.set(y, x, color);  // if transposed, de−transpose
        } else {
            image.set(x, y, color);
        }
        total_dy += k3;
        if (total_dy > dx) {
            y += y1 > y0 ? 1 : -1;
            total_dy -= 2 * dx;
        }
    }
}

void try1()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line1(13, 20, 80, 40, image, white);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line1.tga");
    return;
}
void try2()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line2(13, 20, 80, 40, image, white);
    line2(20, 13, 40, 80, image, red);
    line2(80, 40, 13, 20, image, red);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line2.tga");
    return;
}
void try3()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line3(13, 20, 80, 40, image, white);
    line3(20, 13, 40, 80, image, red);
    line3(80, 40, 13, 20, image, red);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line3.tga");
    return;
}

void try4()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line4(13, 20, 80, 40, image, white);
    line4(20, 13, 40, 80, image, red);
    line4(80, 40, 13, 20, image, red);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line4.tga");
    return;
}

void try5()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line5(13, 20, 80, 40, image, white);
    line5(20, 13, 40, 80, image, red);
    line5(80, 40, 13, 20, image, red);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line5.tga");
    return;
}

void load_model()
{
    // wavefront obj file format
    // v vertex 顶点 格式为 v x y z（坐标
    // f face 面 格式为 f vertex_index(顶点索引) texture_index(材质索引) normal_index(法线索引)
    // v 0.608654 -0.568839 -0.416318
    // 这个顶点的坐标 x,y,z
    // f 1193/1240/1193 1180/1227/1180 1179/1226/1179
    // 前三个，意思是这三个 vertex 连线构成一个 face，序号从1开始，按文件从前往后出现的 v 顺序
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j = 0; j < 3; j++) {
            Vec3f v0 = model->vert(face[j]);
            Vec3f v1 = model->vert(face[(j + 1) % 3]);
            int x0 = (v0.x + 1.0) * width / 2.;
            int y0 = (v0.y + 1.0) * height / 2.;
            int x1 = (v1.x + 1.0) * width / 2.;
            int y1 = (v1.y + 1.0) * height / 2.;
            line5(x0, y0, x1, y1, image, white);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head.tga");
    delete model;
    return;
}

void line6(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color)
{
    line5(t0.x, t0.y, t1.x, t1.y, image, color);
}

// 线扫描法 line sweeping
// 算出起点终点，按起点终点的 x 轴为起止，按线段逐像素扫描
void triangle1(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    if (t0.y == t1.y && t0.y == t2.y) return;  // 为线段时不处理
    // make t0.y <= t1.y <= t2.y
    if (t0.y > t1.y) std::swap(t0, t1);
    if (t0.y > t2.y) std::swap(t0, t2);
    if (t1.y > t2.y) std::swap(t1, t2);
    int total_height = t2.y - t0.y;
    for (int y = t0.y; y < t1.y; y++) {
        int part_height = t1.y == t0.y ? 1 : t1.y - t0.y;  // 防止除 0 异常
        float k02 = static_cast<float>(y - t0.y) / total_height;
        float k01 = static_cast<float>(y - t0.y) / part_height;
        Vec2i a = t0 + (t2 - t0) * k02;
        Vec2i b = t0 + (t1 - t0) * k01;
        if (a.x > b.x) std::swap(a, b);
        for (int x = a.x; x < b.x; x++) {
            image.set(x, y, color);
        }
    }
    for (int y = t1.y; y < t2.y; y++) {
        int part_height = t2.y == t1.y ? 1 : t2.y - t1.y;  // 防止除 0 异常
        float k02 = static_cast<float>(y - t0.y) / total_height;
        float k12 = static_cast<float>(y - t1.y) / part_height;
        Vec2i a = t0 + (t2 - t0) * k02;
        Vec2i b = t1 + (t2 - t1) * k12;
        if (a.x > b.x) std::swap(a, b);
        for (int x = a.x; x < b.x; x++) {
            image.set(x, y, color);
        }
    }
    return;
}

// 计算重心坐标
Vec3f barycentric(Vec2i t0, Vec2i t1, Vec2i t2, Vec2i p)
{
    Vec3f u = cross(Vec3f(t2.x - t0.x, t1.x - t0.x, t0.x - p.x),
                    Vec3f(t2.y - t0.y, t1.y - t0.y, t0.y - p.y));
    // 处理 t0,t1,t2 为直线的 corner case
    // 因 t0,t1,t2,p 都是整型坐标，只可能是整数
    // 因此 abs (u.z) < 1 即 aob(u.z) == 0，即三角形退化为直线，在这种情况下返回负坐标
    if (std::abs(u.z) < 1) return Vec3f(-1, 1, 1);
    return Vec3f(1.0 - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}

// 边界框光栅化 Bounding Box Rasterization
// 逐像素判断其重心坐标是否在三角形内，若是则处理，否则不处理
void triangle2(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    if (t0.y == t1.y && t0.y == t2.y) return;  // 为线段时不处理
    // 计算边界框的范围，左下角起点坐标 box_min，右上角终点坐标 box_max
    Vec2i box_min {image.get_width() - 1, image.get_height() - 1};
    Vec2i box_max {0, 0};
    box_min.x = std::clamp(std::min({box_min.x, t0.x, t1.x, t2.x}), 0, image.get_width() - 1);
    box_min.y = std::clamp(std::min({box_min.y, t0.y, t1.y, t2.y}), 0, image.get_height() - 1);
    box_max.x = std::clamp(std::max({box_max.x, t0.x, t1.x, t2.x}), 0, image.get_width() - 1);
    box_max.y = std::clamp(std::max({box_max.y, t0.y, t1.y, t2.y}), 0, image.get_height() - 1);

    // 遍历边界框内像素点是否在三角形内
    Vec2i p;
    for (p.x = box_min.x; p.x <= box_max.x; p.x++) {
        for (p.y = box_min.y; p.y <= box_max.y; p.y++) {
            Vec3f vec = barycentric(t0, t1, t2, p);
            if (vec.x < 0 || vec.y < 0 || vec.z < 0) continue;
            image.set(p.x, p.y, color);
        }
    }
    return;
}

void lesson2_try1()
{
    TGAImage image(200, 200, TGAImage::RGB);
    Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
    Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
    Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
    triangle1(t0[0], t0[1], t0[2], image, red);
    triangle1(t1[0], t1[1], t1[2], image, white);
    triangle1(t2[0], t2[1], t2[2], image, green);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line_sweeping.tga");
    return;
}

void lesson2_try2()
{
    TGAImage image(200, 200, TGAImage::RGB);
    Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
    Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
    Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
    triangle2(t0[0], t0[1], t0[2], image, red);
    triangle2(t1[0], t1[1], t1[2], image, white);
    triangle2(t2[0], t2[1], t2[2], image, green);
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("bounding_box.tga");
    return;
}

// 加载模型，每个三角形面使用随机颜色绘制
void load_model_and_random_color()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i vertex[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v = model->vert(face[j]);
            int x = (v.x + 1.0) * width / 2.;
            int y = (v.y + 1.0) * height / 2.;
            vertex[j] = {x, y};
        }
        TGAColor color;
        color.bgra[0] = rand() % 255;
        color.bgra[1] = rand() % 255;
        color.bgra[2] = rand() % 255;
        color.bgra[3] = 255;
        triangle2(vertex[0], vertex[1], vertex[2], image, color);
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_random_color.tga");
    delete model;
    return;
}

// 添加平行光，计算每个三角形面的光强度
void load_model_and_light()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    // 定义一个平行光，照向 -z 方向
    Vec3f light_dir {0, 0, -1};
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v = model->vert(face[j]);
            int x = (v.x + 1.0) * width / 2.0;
            int y = (v.y + 1.0) * height / 2.0;
            screen_coords[j] = {x, y};
            world_coords[j] = v;
        }
        // 计算三角形面的法向量
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n.normalize();
        // 计算法向量与平行光的夹角，决定了三角形面的亮度
        float intensity = n * light_dir;
        // 如果点乘大于 0，光源对该平面的照射方向与法向量同向
        // 说明光源对该平面的照射方向是从后往前的，因此忽略不处理，正常而言，你看不见物体背后的光照效果
        if (intensity > 0) {
            TGAColor color;
            color.bgra[0] = intensity * 255;
            color.bgra[1] = intensity * 255;
            color.bgra[2] = intensity * 255;
            color.bgra[3] = 255;
            triangle2(screen_coords[0], screen_coords[1], screen_coords[2], image, color);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_light.tga");
    delete model;
    return;
}

// z_buffer
// 记录像素点的 z 轴深度信息从而处理遮挡关系
void triangle3(Vec2i screen_coords[3], Vec3f world_coords[3],
               std::vector<std::vector<float>> &z_buffer, TGAImage &image, TGAColor color)
{
    const auto &sc = screen_coords;
    if (sc[0].y == sc[1].y && sc[0].y == sc[2].y) return;  // 为线段时不处理
    // 计算边界框的范围，左下角起点坐标 box_min，右上角终点坐标 box_max
    Vec2i box_min {image.get_width() - 1, image.get_height() - 1};
    Vec2i box_max {0, 0};
    box_min.x =
        std::clamp(std::min({box_min.x, sc[0].x, sc[1].x, sc[2].x}), 0, image.get_width() - 1);
    box_min.y =
        std::clamp(std::min({box_min.y, sc[0].y, sc[1].y, sc[2].y}), 0, image.get_height() - 1);
    box_max.x =
        std::clamp(std::max({box_max.x, sc[0].x, sc[1].x, sc[2].x}), 0, image.get_width() - 1);
    box_max.y =
        std::clamp(std::max({box_max.y, sc[0].y, sc[1].y, sc[2].y}), 0, image.get_height() - 1);

    // 遍历边界框内像素点是否在三角形内
    Vec2i p;
    const auto &wc = world_coords;
    for (p.x = box_min.x; p.x <= box_max.x; p.x++) {
        for (p.y = box_min.y; p.y <= box_max.y; p.y++) {
            Vec3f vec = barycentric(sc[0], sc[1], sc[2], p);
            if (vec.x < 0 || vec.y < 0 || vec.z < 0) continue;
            // 计算重心坐标的 z
            float z = vec.x * wc[0].z + vec.y * wc[1].z + vec.z * wc[2].z;
            // 只处理该像素点 z 轴最前面的三角形面，即重心坐标 z 最大的三角形面
            if (z < z_buffer[p.x][p.y]) continue;
            z_buffer[p.x][p.y] = z;
            image.set(p.x, p.y, color);
        }
    }
    return;
}

void load_model_and_z_buffer()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    // 有个小坑，浮点数的 min() 返回的是最小正数，得用 lowest() 才是负的最小值
    float lowest = std::numeric_limits<float>::lowest();
    // 记录像素点 z 轴深度信息
    std::vector<std::vector<float>> z_buffer(width, std::vector<float>(height, lowest));
    // 定义一个平行光，照向 -z 方向
    Vec3f light_dir {0, 0, -1};
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v = model->vert(face[j]);
            int x = (v.x + 1.0) * width / 2.0;
            int y = (v.y + 1.0) * height / 2.0;
            screen_coords[j] = {x, y};
            world_coords[j] = v;
        }
        // 计算三角形面的法向量
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n.normalize();
        // 计算法向量与平行光的夹角，决定了三角形面的亮度
        float intensity = n * light_dir;
        // 如果点乘大于 0，光源对该平面的照射方向与法向量同向
        // 说明光源对该平面的照射方向是从后往前的，因此忽略不处理，正常而言，你看不见物体背后的光照效果
        if (intensity > 0) {
            TGAColor color;
            color.bgra[0] = intensity * 255;
            color.bgra[1] = intensity * 255;
            color.bgra[2] = intensity * 255;
            color.bgra[3] = 255;
            triangle3(screen_coords, world_coords, z_buffer, image, color);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_z_buffer.tga");
    delete model;
    return;
}

constexpr int dim = 4;
template <typename T>
class Matrix4 {
public:
    Matrix4() {}
    Matrix4(const std::initializer_list<std::initializer_list<T>> &init_list)
    {
        *this = init_list;  // 复用 operator=()
    }
    void init_E()
    {
        for (int i = 0; i < dim; i++)
            for (int j = 0; j < dim; j++) {
                m_[i][j] = (i == j ? 1.0f : 0.0f);
            }
    }
    Matrix4 &operator=(const std::initializer_list<std::initializer_list<T>> &init_list)
    {
        int i = 0;
        for (const auto &item : init_list) {
            int j = 0;
            for (const auto &it : item) {
                m_[i][j] = static_cast<T>(it);
                j++;
            }
            i++;
        }
        return *this;
    }
    auto &operator[](int i) { return m_[i]; }
    const auto &operator[](int i) const { return m_[i]; }
    template <typename V>
    V operator*(const V &vec)
    {
        V ret;
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                ret[i] += m_[i][j] * vec[j];
            }
        }
        return ret;
    }
    Matrix4 operator*(const Matrix4 &rhs)
    {
        Matrix4 ret;
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                ret[i][j] = 0.0f;
                for (int k = 0; k < dim; k++) {
                    ret[i][j] += m_[i][k] * rhs[k][j];
                }
            }
        }
        return ret;
    }
    template <typename U>
    friend std::ostream &operator<<(std::ostream &s, Matrix4<U> &rhs);

private:
    std::array<std::array<T, dim>, dim> m_;
};

template <typename U>
std::ostream &operator<<(std::ostream &cout, Matrix4<U> &rhs)
{
    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < dim; j++) {
            std::cout << rhs.m_[i][j] << ", ";
        }
        std::cout << std::endl;
    }
    return cout;
}

using Matrix4f = Matrix4<float>;

Matrix4f get_model_trans()
{
    // 缩放: x,y 缩放 0.5
    float k = 0.5;
    Matrix4f scale = {{k, 0, 0, 0}, {0, k, 0, 0}, {0, 0, k, 0}, {0, 0, 0, 1}};
    // 旋转: 绕 y 轴逆时针旋转 90 度
    Matrix4f rotate = {{0, 0, 1, 0}, {0, 1, 0, 0}, {-1, 0, 0, 0}, {0, 0, 0, 1}};
    // 平移: x 轴平移 0.5
    Matrix4f translation = {{1, 0, 0, 0.5}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    // 注意，变换顺序不同，结果不同
    Matrix4f model_trans = translation * rotate * scale;
    return model_trans;
}

// 模型变换
void model_transformation()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    Matrix4f model_trans = get_model_trans();

    float lowest = std::numeric_limits<float>::lowest();
    std::vector<std::vector<float>> z_buffer(width, std::vector<float>(height, lowest));
    // 定义一个平行光，照向 -z 方向
    Vec3f light_dir {0, 0, -1};
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v_origin = model->vert(face[j]);
            // 转化为齐次坐标
            Vec4f v_homo = Vec4f {v_origin.x, v_origin.y, v_origin.z, 1.0f};
            // 模型变换，将物体放置在合适的位置
            Vec4f v_model = model_trans * v_homo;
            int x = (v_model.x + 1.0) * width / 2.0;
            int y = (v_model.y + 1.0) * height / 2.0;
            screen_coords[j] = {x, y};
            world_coords[j] = {v_model.x, v_model.y, v_model.z};
        }
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        // 如果点乘大于 0，光源对该平面的照射方向与法向量同向
        // 说明光源对该平面的照射方向是从后往前的，因此忽略不处理，正常而言，你看不见物体背后的光照效果
        if (intensity > 0) {
            TGAColor color;
            color.bgra[0] = intensity * 255;
            color.bgra[1] = intensity * 255;
            color.bgra[2] = intensity * 255;
            color.bgra[3] = 255;
            triangle3(screen_coords, world_coords, z_buffer, image, color);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_model_trans.tga");
    delete model;
    return;
}

Matrix4f get_view_trans(Vec3f camera, Vec3f front, Vec3f up)
{
    // 注意这里是 面朝方向 叉乘 头顶方向，得到摄像机右手方向
    // 因此摄像机的坐标系基向量为（右，上，前），需将这三个基向量分别对齐 x轴，y轴，-z轴
    Vec3f right = cross(front, up);  // x

    // 摄像机位于 (1,0,0)，看向 -x 方向，头顶向 +y 方向
    // Vec3f camera = {1, 0, 0};
    // Vec3f front = {-1, 0, 0};  // -z
    // Vec3f up = {0, 1, 0};      // y
    // Vec3f right = front ^ up;   // x

    // 摄像机位于 (-1,0,0)，看向 +x 方向，头顶向 +y 方向
    // Vec3f camera = {-1, 0, 0};
    // Vec3f front = {1, 0, 0};  // -z
    // Vec3f up = {0, 1, 0};     // y
    // Vec3f right = front ^ up;   // x

    // 摄像机位于 (0,0,1)，看向 -z 方向，头顶向 +y 方向
    // Vec3f camera = {0, 0, 1};
    // Vec3f front = {0, 0, -1};  // -z
    // Vec3f up = {0, 1, 0};      // y
    // Vec3f right = front ^ up;   // x

    // 摄像机位于 (1,1,1)，看向 (-1,-1,-1) 方向，头顶向 (-1,1,-1) 方向
    // Vec3f camera = {1, 1, 1};
    // Vec3f front = {-1, -1, -1};  // -z
    // Vec3f up = {-1, 1, -1};      // y
    // Vec3f right = front ^ up;  // x

    // 单位化为单位向量
    front.normalize();
    up.normalize();
    right.normalize();

    // 平移: 将摄像机移动至原点
    Matrix4f view_t = {
        {1.0f, 0, 0, -camera.x}, {0.0f, 1, 0, -camera.y}, {0.0f, 0, 1, -camera.z}, {0.0f, 0, 0, 1}};
    // 旋转: 将摄像机坐标系的基向量对齐基向量
    // 先写出 x,y,z 轴基向量旋转为摄像机坐标系基向量的旋转矩阵
    // 注意各基向量对齐对应的向量 x->right, y->up，-z->front
    // Matrix4f view_r_1 = {{right.x, up.x, -front.x, 0},
    //                    {right.y, up.y, -front.y, 0},
    //                    {right.z, up.z, -front.z, 0},
    //                    {0, 0, 0, 1}};
    // 求逆，对于正交矩阵，求逆等同于求其转置矩阵，因此 view_r = view_r_1 的转置
    Matrix4f view_r = {{right.x, right.y, right.z, 0},
                       {up.x, up.y, up.z, 0},
                       {-front.x, -front.y, -front.z, 0},
                       {0, 0, 0, 1}};
    Matrix4f view_trans = view_r * view_t;
    return view_trans;
}

// 视角变换，摄像机变换
void view_transformation()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    Matrix4f model_trans = get_model_trans();

    // 摄像机位于 (1,1,1)，看向 (-1,-1,-1) 方向，头顶向 (-1,1,-1) 方向
    Vec3f camera = {1, 1, 1};    // x
    Vec3f front = {-1, -1, -1};  // z
    Vec3f up = {-1, 1, -1};      // y
    Matrix4f view_trans = get_view_trans(camera, front, up);

    float lowest = std::numeric_limits<float>::lowest();
    std::vector<std::vector<float>> z_buffer(width, std::vector<float>(height, lowest));
    // 定义一个平行光，照向 -z 方向
    Vec3f light_dir = {0, 0, -1};  // 这个函数中，视角变换并没有计算并改变 light_dir
                                   // 这也就意味着经过变换后计算光照时，始终是从摄像机背后照向前方
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v_origin = model->vert(face[j]);
            // 转化为齐次坐标
            Vec4f v_homo = Vec4f {v_origin.x, v_origin.y, v_origin.z, 1.0f};
            // 模型变换，将物体放置在合适的位置
            Vec4f v_model = model_trans * v_homo;
            // 视图变换，从摄像机角度看到的视角
            Vec4f v_view = view_trans * v_model;
            int x = (v_view.x + 1.0) * width / 2.0;
            int y = (v_view.y + 1.0) * height / 2.0;
            screen_coords[j] = {x, y};
            world_coords[j] = {v_view.x, v_view.y, v_view.z};
        }
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        // 如果点乘大于 0，光源对该平面的照射方向与法向量同向
        // 说明光源对该平面的照射方向是从后往前的，因此忽略不处理，正常而言，你看不见物体背后的光照效果
        if (intensity > 0) {
            TGAColor color;
            color.bgra[0] = intensity * 255;
            color.bgra[1] = intensity * 255;
            color.bgra[2] = intensity * 255;
            color.bgra[3] = 255;
            triangle3(screen_coords, world_coords, z_buffer, image, color);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_view_trans.tga");
    delete model;
    return;
}

// 透视投影稍微有点麻烦，等后面有空在写
// void project_transformation() {}

constexpr int depth = 255;
Matrix4f get_viewport_trans(int x, int y, int w, int h)
{
    Matrix4f viewport_trans;
    viewport_trans.init_E();
    // 平移
    viewport_trans[0][3] = x + w / 2.f;
    viewport_trans[1][3] = y + h / 2.f;
    viewport_trans[2][3] = depth / 2.f;
    // 缩放
    viewport_trans[0][0] = w / 2.f;
    viewport_trans[1][1] = h / 2.f;
    viewport_trans[2][2] = depth / 2.f;
    return viewport_trans;
}

// 视口变换
void viewport_transformation()
{
    Model *model = new Model("obj/african_head.obj");
    constexpr int width = 800;
    constexpr int height = 800;
    TGAImage image(width, height, TGAImage::RGB);

    Matrix4f model_trans;
    model_trans.init_E();
    // 摄像机位于 (0,0,1)，看向 -z 方向，头顶向 +y 方向
    Vec3f camera = {0, 0, 1};  // x
    Vec3f front = {0, 0, -1};  // -z
    Vec3f up = {0, 1, 0};      // y
    Matrix4f view_trans = get_view_trans(camera, front, up);
    Matrix4f viewport_trans =
        get_viewport_trans(width / 8, height / 8, width * 3 / 4, height * 3 / 4);

    float lowest = std::numeric_limits<float>::lowest();
    std::vector<std::vector<float>> z_buffer(width, std::vector<float>(height, lowest));
    Vec3f light_dir = {0, 0, -1};
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v_origin = model->vert(face[j]);
            // 转化为齐次坐标
            Vec4f v_homo = Vec4f {v_origin.x, v_origin.y, v_origin.z, 1.0f};
            // 模型变换，将物体放置在合适的位置
            Vec4f v_model = model_trans * v_homo;
            // 视图变换，从摄像机角度看到的视角
            Vec4f v_view = view_trans * v_model;
            // 视口变换，映射到屏幕像素
            Vec4f v_viewport = viewport_trans * v_view;
            screen_coords[j] = {(int)v_viewport.x, (int)v_viewport.y};
            world_coords[j] = {v_view.x, v_view.y, v_view.z};
        }
        Vec3f n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0) {
            TGAColor color;
            color.bgra[0] = intensity * 255;
            color.bgra[1] = intensity * 255;
            color.bgra[2] = intensity * 255;
            color.bgra[3] = 255;
            triangle3(screen_coords, world_coords, z_buffer, image, color);
        }
    }
    image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("african_head_viewport_trans.tga");
    delete model;
    return;
}

int main(int argc, char **argv)
{
    // lesson 1
    try1();
    try2();
    try3();
    try4();
    try5();
    load_model();
    // lesson 2
    lesson2_try1();
    lesson2_try2();
    load_model_and_random_color();
    load_model_and_light();
    // lesson 3
    load_model_and_z_buffer();
    // lession 4
    model_transformation();
    view_transformation();
    // project_transformation();
    viewport_transformation();
    return 0;
}
