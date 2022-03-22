#include "include/UE4Helper.h"
#include "include/RpcClientGetter.h"
#include <iostream>
#pragma warning(disable : 4189)
#pragma warning(disable : 4305)

namespace AirSimTools
{
/// @brief 利用旋转矩阵修正从X-Plane到AirSim的坐标
/// @param flight_data 来自X-Plane的NED坐标
/// @param pitch_error 俯仰角修正量
/// @param roll_error 滚转角修正量
/// @param psi_error 偏航角修正量
/// @return 修正过后的AirSim的NED坐标
/// @note AirSim的以PlyerStart初始状态定义NED坐标系，此函数类似于进行了机体坐标系转地面坐标系的操作
FlightData UE4Helper::AdaptPosition(const FlightData& flight_data, float pitch_modified, float roll_modified, float yaw_modified)
{
    FlightData flight_data_; //修正过后的AirSim的NED坐标
    float theta_ = pitch_modified / 57.3f;
    float phi_ = roll_modified / 57.3f;
    float psi_ = yaw_modified / 57.3f;
    bool ONLY_PSI = 1; // 1 修正psi偏差模式 0 其他模式
    if (ONLY_PSI) {
        flight_data_.position[0] = flight_data.position[0] * cos(psi_) - flight_data.position[1] * sin(psi_);
        flight_data_.position[1] = flight_data.position[0] * sin(psi_) + flight_data.position[1] * cos(psi_);
        flight_data_.position[2] = flight_data.position[2];
        flight_data_.attitude[0] = flight_data.attitude[0];
        flight_data_.attitude[1] = flight_data.attitude[1];
        flight_data_.attitude[2] = flight_data.attitude[2] + psi_;
        flight_data_.orientation = msr::airlib::VectorMath::toQuaternion(flight_data_.attitude[0], flight_data_.attitude[1], flight_data_.attitude[2]);
    }
    else {
        //float x = flight_data.position[0];
        //float y = flight_data.position[1];
        //float z = flight_data.position[2];
        //flight_data_.position[0] = x * cos(theta_) * cos(psi_) + y * (sin(phi_) * sin(theta_) * cos(psi_) - cos(phi_) * sin(psi_)) + z * (sin(phi_) * sin(psi_) + cos(phi_) * sin(theta_) * cos(psi_));
        //flight_data_.position[1] = x * cos(theta_) * sin(psi_) + y * (sin(phi_) * sin(theta_) * sin(psi_) + cos(phi_) * cos(psi_)) + z * (-sin(phi_) * cos(psi_) + cos(phi_) * sin(theta_) * sin(psi_));
        //flight_data_.position[2] = -x * sin(theta_) + y * sin(phi_) * cos(theta_) + z * cos(phi_) * cos(theta_);
        //flight_data_.attitude[0] = flight_data.attitude[0] + theta_;
        //flight_data_.attitude[1] = flight_data.attitude[1] + phi_;
        //flight_data_.attitude[2] = flight_data.attitude[2] + psi_;
        flight_data_.position[0] = flight_data.position[0];
        flight_data_.position[1] = flight_data.position[1];
        flight_data_.position[2] = flight_data.position[2];
        flight_data_.attitude[0] = flight_data.attitude[0];
        flight_data_.attitude[1] = flight_data.attitude[1];
        flight_data_.attitude[2] = flight_data.attitude[2];
        flight_data_.orientation = msr::airlib::VectorMath::toQuaternion(flight_data_.attitude[0], flight_data_.attitude[1], flight_data_.attitude[2]);
        //std::cout << "FlightData updated =" << flight_data_ << std::endl;
    }
    return flight_data_;
}

void UE4Helper::UpdatePosition(const FlightData& flight_data)
{
    msr::airlib::Pose current_pose = msr::airlib::Pose(msr::airlib::Vector3r((float)flight_data.position[0], (float)flight_data.position[1], (float)flight_data.position[2]), flight_data.orientation);
    RpcClientGetter::Get().simSetVehiclePose(current_pose, true);
}

CameraData UE4Helper::getCameraInfo(const std::string& camera_name)
{
    CameraData camera_data;
    //CameraInfo CameraInfo_;
    //CameraInfo_ = RpcClientGetter::Get().simGetCameraInfo("ExternalCamera");
    //std::cout << "fov = " << CameraInfo_.fov << std::endl;
    //std::cout << "position = " << CameraInfo_.pose.position << std::endl;
    //std::cout << "orientation = " << CameraInfo_.pose.orientation << std::endl;
    Pose Pose_;
    msr::airlib::real_T pitch_; // float
    msr::airlib::real_T roll_;
    msr::airlib::real_T yaw_;
    Pose_ = RpcClientGetter::Get().simGetObjectPose(camera_name);
    msr::airlib::VectorMath::toEulerianAngle(Pose_.orientation, pitch_, roll_, yaw_);
    camera_data.position[0] = Pose_.position[0];
    camera_data.position[1] = Pose_.position[1];
    camera_data.position[2] = Pose_.position[2];
    camera_data.attitude[0] = pitch_;
    camera_data.attitude[1] = roll_;
    camera_data.attitude[2] = yaw_;
    camera_data.orientation.x() = Pose_.orientation.x();
    camera_data.orientation.y() = Pose_.orientation.y();
    camera_data.orientation.z() = Pose_.orientation.z();
    camera_data.orientation.w() = Pose_.orientation.w();
    //std::cout << "position = " << Pose_.position << std::endl;
    //std::cout << "orientation = " << Pose_.orientation << std::endl;
    return camera_data;
}

UE4Helper::ImageCaptureResult UE4Helper::CaptureGroundCameraImage(CameraMode camera_mode)
{
    ImageCaptureResult image_capture_result;
    std::vector<ImageRequest> request;
    msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get(); //工厂模式，仅初始化一次clock，功能与单例模式类似
    auto start_nanos = clock->nowNanos(); //开始计时
    switch (camera_mode) {
    case CameraMode::origin_mode:
        request.push_back(ImageRequest("front_left", ImageType::Scene, false, false));
        image_capture_result.response = RpcClientGetter::Get().simGetImages(request); // 使用rpc获取图片
        break;
    case CameraMode::ground_mode:
        request.push_back(ImageRequest("ExternalCamera", ImageType::Scene, false, false));
        image_capture_result.response = RpcClientGetter::Get().simExternalImages(request);
        break;
    }
    double render_time_temp = clock->elapsedSince(start_nanos); //停止计时
    //std::cout << "render_time: " << render_time_temp * 1E3f << " ms" << std::endl; // 显示渲染时间（图片获取时间）
    image_capture_result.render_time = render_time_temp;

    return image_capture_result;
}

void UE4Helper::ProcessCameraImage(ImageCaptureResult& image_capture_result)
{
    msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get(); //工厂模式，仅初始化一次clock，功能与单例模式类似
    auto start_nanos = clock->nowNanos();
    const auto& image_data = image_capture_result.response.at(0).image_data_uint8;
    double render_time_temp = clock->elapsedSince(start_nanos);
    //std::cout << "render time = " << render_time_temp * 1E3f << " ms" << std::endl;
}

std::vector<uchar> UE4Helper::ImageEncoder(ImageCaptureResult& image_capture_result)
{
    int Width = image_capture_result.response.at(0).width;
    int Height = image_capture_result.response.at(0).height;
    std::vector<uchar> encoded_image(Width * Height * 3);
    cv::Mat cv_image(Height, Width, CV_8UC(4));
    int idx = 0;
    for (int x = 0; x < Height; x++) {
        for (int y = 0; y < Width; y++) {
            cv_image.at<cv::Vec4b>(x, y)[0] = image_capture_result.response.at(0).image_data_uint8[idx]; //B
            cv_image.at<cv::Vec4b>(x, y)[1] = image_capture_result.response.at(0).image_data_uint8[idx + 1]; //G
            cv_image.at<cv::Vec4b>(x, y)[2] = image_capture_result.response.at(0).image_data_uint8[idx + 2]; //R
            cv_image.at<cv::Vec4b>(x, y)[3] = 0; //alpha
            idx += 3;
        }
    }
    vector<int> encode_param = vector<int>(2);
    encode_param[0] = CV_IMWRITE_JPEG_QUALITY;
    encode_param[1] = 70;
    if (cv_image.data) //防止图片内容为空，导致opencv错误
    {
        //cv::imshow("pic-origin", cv_image);
        //cv::waitKey(1);
        cv::imencode(".jpg", cv_image, encoded_image, encode_param);
    }
    return encoded_image;
}
}
