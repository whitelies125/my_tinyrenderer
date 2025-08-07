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
    // image.flip_vertically();  // 垂直翻转 左下角为原点
    image.write_tga_file("line1.tga");
    return;
}
void try2()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line2(13, 20, 80, 40, image, white);
    line2(20, 13, 40, 80, image, red);
    line2(80, 40, 13, 20, image, red);
    image.write_tga_file("line2.tga");
    return;
}
void try3()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line3(13, 20, 80, 40, image, white);
    line3(20, 13, 40, 80, image, red);
    line3(80, 40, 13, 20, image, red);
    image.write_tga_file("line3.tga");
    return;
}

void try4()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line4(13, 20, 80, 40, image, white);
    line4(20, 13, 40, 80, image, red);
    line4(80, 40, 13, 20, image, red);
    image.write_tga_file("line4.tga");
    return;
}

void try5()
{
    TGAImage image(100, 100, TGAImage::RGB);
    line5(13, 20, 80, 40, image, white);
    line5(20, 13, 40, 80, image, red);
    line5(80, 40, 13, 20, image, red);
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
    image.write_tga_file("african_head.tga");
    delete model;
    return;
}
int main(int argc, char **argv)
{
    try1();
    try2();
    try3();
    try4();
    try5();
    load_model();
    return 0;
}
