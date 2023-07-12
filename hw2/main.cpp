// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"
#include<Eigen/Dense>
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float a = rotation_angle / 180 * MY_PI;
    model << cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection;
    float n=zNear,  f=zFar;
    //͸�ӱ伷ѹ
    Eigen::Matrix4f Mpersp_orhtho;
    Mpersp_orhtho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    //��ȡl,r,n,f,t,b
    float fov = eye_fov / 180 * MY_PI;
    float t = (-n) * tan(fov / 2), b = -t;
    float r = t * aspect_ratio, l = -r;
    ////��������
    Eigen::Matrix4f orthoscale,orthomove;
    orthoscale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    orthomove << 1, 0, 0, -(r + l) / 2, 0, 1, 0, -(t + b) / 2, 0, 0, 1, -(n + f) / 2, 0, 0, 0, 1;
    projection =   orthoscale*orthomove* Mpersp_orhtho;
    return projection;

    //Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //// TODO: Implement this function
    //// Create the projection matrix for the given parameters.
    //// Then return it.

    //float n = zNear, f = zFar;

    //// ͸��ͶӰ->����ͶӰ  ��ѹ
    //Eigen::Matrix4f Mpersp_orhtho;
    //Mpersp_orhtho << n, 0, 0, 0,
    //    0, n, 0, 0,
    //    0, 0, n + f, -n * f,
    //    0, 0, 1, 0;

    //// ����ͶӰ->����������
    //    // ����׶��ϢΪr,l,t,b
    //float fovY = eye_fov / 180 * MY_PI;// �Ƕ�ת����
    //float t = tan(fovY / 2) * (-n), b = -t;// ����-z����|n|
    //float r = aspect_ratio * t, l = -r;
    // ת��������������
    //Eigen::Matrix4f Mortho, Mtrans, Mscale;
    //Mtrans << 1, 0, 0, -(r + l) / 2,
    //    0, 1, 0, -(t + b) / 2,
    //    0, 0, 1, -(n + f) / 2,
    //    0, 0, 0, 1;
    //Mscale << 2 / (r - l), 0, 0, 0,
    //    0, 2 / (t - b), 0, 0,
    //    0, 0, 2 / (n - f), 0,
    //    0, 0, 0, 1;
    //Mortho = Mscale * Mtrans;

    //// ����õ�ͶӰ����
    //projection = Mortho * Mpersp_orhtho;
    //return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on