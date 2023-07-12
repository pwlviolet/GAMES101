#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
constexpr double MY_PI = 3.1415926;
//平移操作
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}
//二维绕x轴旋转的方法
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();


    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    //转化为弧度，绕z旋转
    float a = rotation_angle / 180 * MY_PI;
    model << cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    //绕x轴旋转
    //model << 1, 0, 0, 0,
    //        0, cos(a), -sin(a), 0,
    //        0, sin(a), cos(a), 0,
    //        0, 0, 0, 1;
    //绕y轴旋转
    //model << cos(a), 0, sin(a), 0,
    //    0, 1, 0, 0,
    //    -sin(a), 0, cos(a), 0,
    //    0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = zNear, f = zFar;

    // 透视投影->正交投影  挤压
    Eigen::Matrix4f Mpersp_orhtho;
    Mpersp_orhtho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // 正交投影->正则立方体
        // 将视锥信息为r,l,t,b
    float fovY = eye_fov / 180 * MY_PI;// 角度转弧度
    float t = tan(fovY / 2) * (-n), b = -t;// 朝向-z方向|n|
    float r = aspect_ratio * t, l = -r;
    // 转换到正则立方体
    Eigen::Matrix4f Mortho, Mtrans, Mscale;
    Mtrans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Mscale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    Mortho = Mscale * Mtrans;

    // 计算得到投影矩阵
    projection = Mortho * Mpersp_orhtho;
    return projection;
}

int main()
{
    //int v1 = 0, v2 = 10;
    //cout << "the sum of" << v1
    //<< "and" << v2
    //<< "is" << v1 + v2 << endl;
    //cout << "/*" << endl;
    //cout << "num" << endl;
    int a, b ;
    cin>>a>>b;
    cout << a + b << endl;
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    //if (argc >= 3) {
    //    command_line = true;
    //    angle = std::stof(argv[2]); // -r by default
    //    if (argc == 4) {
    //        filename = std::string(argv[3]);
    //    }
    //}
    //创建渲染器的窗口
    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
