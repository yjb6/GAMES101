#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto sita=rotation_angle/180*MY_PI;
    model(0,0)=cos(sita);
    model(0,1)=-sin(sita);
    model(1,0)=sin(sita);
    model(1,1)=cos(sita);
    return model;
}
Eigen::Matrix4f get_rotation(Vector3f axis,float angle)
{
    float sita=angle/180*MY_PI;
    Eigen::MatrixXf R = Eigen::Matrix3f::Identity();
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f t;
    // Eigen::Vector3f n;
    // n<<axis[0],axis[1],axis[2],0;
    t<<0,-axis[2],axis[1],axis[2],0,-axis[0],-axis[1],axis[0],0;
    R=R*cos(sita)+(1-cos(sita))*axis*axis.transpose()+sin(sita)*t;
    // std::cout<<R<<std::endl;
    // model<<R(0,0),R(0,0),R(0,0),R(0,0),R(0,0),R(0,0),R(0,0),R(0,0),R(0,0),R(0,0);
    Eigen::VectorXf v(3);
    v<<0,0,0;
    // std::cout<<v<<"\n";
    R.conservativeResize(R.rows(),R.cols()+1);
    R.col(R.cols()-1)=v;
    R.conservativeResize(R.rows()+1,R.cols());
    v.resize(4);
    v[3]=1;
    // std::cout<<v<<"\n";
    R.row(R.rows()-1)=v.transpose();
    // model+=R;
    // std::cout<<R;
    // R(3,0)=0;
    return R;
}
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();
    // float z=abs(zNear);
    persp2ortho(0,0)=-zNear;
    persp2ortho(1,1)=-zNear;
    persp2ortho(3,3)=0;
    persp2ortho(3,2)=1;
    persp2ortho(2,2)=-(zNear+zFar);
    persp2ortho(2,3)=-zFar*zNear;
    auto sita=eye_fov/180*MY_PI;
    float t=tan(sita/2)*zNear;
    float r=t*aspect_ratio;
    ortho(0,0)=1/r;
    ortho(1,1)=1/t;
    ortho(2,2)=1/(zFar-zNear);
    projection=ortho*persp2ortho;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    Eigen::Vector3f n = {0, 0, 1};
    if (argc >= 3) {
        command_line = true;

        if(argc>=6)
        {
            n<<std::stof(argv[2]),std::stof(argv[3]),std::stof(argv[4]);
            n/=sqrt(n.dot(n));
            angle=std::stof(argv[5]);
            if (argc == 7) {
                filename = std::string(argv[6]);
            }
        }
        else{
            angle = std::stof(argv[2]); // -r by default
            if (argc == 4) {
                filename = std::string(argv[3]);
            }
        }
        // else
        //     return 0;
    }

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

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(n,angle));
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
