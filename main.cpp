#include <iostream>
#include <string>
#include <sys/stat.h>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "common/common_utils/FileSystem.hpp"
#include "include/CalcuAnchorPoint.h"
#include "include/LBH2NED.h"
#include "include/Network.h"
#include "include/RpcClientGetter.h"
#include "include/UE4Helper.h"
#include <chrono>
#include <math.h>
#include <thread>

using UE4Helper = AirSimTools::UE4Helper;
using Network = AirSimTools::Network;
using RpcClientGetter = AirSimTools::RpcClientGetter;
using BLH2XYZ = AirSimTools::BLH2XYZ;
using FlightData = AirSimTools::FlightData;
using CameraData = AirSimTools::CameraData;
using XPlaneData = AirSimTools::XPlaneData;
using FileSystem = common_utils::FileSystem;
using AnchorPoint = AirSimTools::CalcuAnchorPoint;

void matlab_flight_data_mode(Network network, std::shared_ptr<FlightData> flight_data, std::shared_ptr<std::mutex> mtx_ptr)
{
    std::vector<char> recv_buff(185, 0); // 5 + 5 * 9 * 4 = 185
    while (1) {
        std::lock_guard<std::mutex> guard(*mtx_ptr);
        network.UDPReceive("127.0.0.1", 10086, recv_buff);
        *flight_data = network.DecodeByteMATLAB(recv_buff);
        std::cout << "FlightData received =" << *flight_data << std::endl;
    }
}

void xplane_flight_data_mode(UE4Helper ue4_helper, Network network, std::shared_ptr<FlightData> flight_data, std::shared_ptr<FlightData> flight_data_updated, std::shared_ptr<std::mutex> mtx_ptr)
{
    BLH2XYZ blh2xyz;
    std::vector<char> recv_buff(185, 0); // 5 + 5 * 9 * 4 = 185
    std::vector<char> send_buff_QT(53, 0);
    FlightData flight_data_this;
    while (1) {
        std::lock_guard<std::mutex> guard(*mtx_ptr);
        network.UDPReceive("127.0.0.1", 49005, recv_buff); //接收X-Plane数据
        network.UDPSend("127.0.0.1", 49006, recv_buff); //转发至matlab
        flight_data_this = blh2xyz.BLH2NED(network.DecodeByteXPlane(recv_buff));
        if (abs(flight_data_this.position[0] - flight_data->position[0]) < 0.0001 ||
            abs(flight_data_this.position[1] - flight_data->position[1]) < 0.0001)
            continue;
        else {
            *flight_data = flight_data_this;
        }
        *flight_data_updated = ue4_helper.AdaptPosition(*flight_data);
        ue4_helper.UpdatePosition(*flight_data_updated); //更新AirSim无人机位姿
    }
}

void update_position(UE4Helper ue4_helper, std::shared_ptr<FlightData> flight_data, std::shared_ptr<std::mutex> mtx_ptr)
{
    while (1) {
        std::lock_guard<std::mutex> guard(*mtx_ptr);
        ue4_helper.UpdatePosition(ue4_helper.AdaptPosition(*flight_data));
        std::cout << "FlightData updated =" << *flight_data << std::endl;
    }
}

void update_position_local_mode(UE4Helper ue4_helper, std::shared_ptr<FlightData> flight_data_updated, std::shared_ptr<std::mutex> mtx_ptr)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float phi = 0.0f;
    float theta = 0.0f;
    float psi = 0.0f;
    float t = 0.0f;

    while (1) {
        std::lock_guard<std::mutex> guard(*mtx_ptr);
        //msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get(); //初始化clock
        //auto start_nanos = clock->nowNanos(); //开始计时
        t += 0.015f;
        x = 300.0f * sin(0.05f * t);
        y = 300.0f * cos(0.05f * t);
        z = 25.0f + 2.0f * sin(0.05f * t);
        phi = 2.0f * 3.14f * sin(0.05f * t);
        theta = 2.0f * 3.14f * sin(0.05f * t);
        psi = 2.0f * 3.14f * sin(0.05f * t);
        flight_data_updated->position[0] = x;
        flight_data_updated->position[1] = y;
        flight_data_updated->position[2] = -z;
        flight_data_updated->attitude[0] = phi;
        flight_data_updated->attitude[1] = theta;
        flight_data_updated->attitude[2] = psi;
        flight_data_updated->orientation = msr::airlib::VectorMath::toQuaternion(flight_data_updated->attitude[0], flight_data_updated->attitude[1], flight_data_updated->attitude[2]);
        ue4_helper.UpdatePosition(*flight_data_updated);
        //double render_time_temp = clock->elapsedSince(start_nanos); //停止计时
        //std::cout << "间隔时间: " << render_time_temp * 1E3f << " ms" << std::endl; // 显示时间
        //std::cout << "FlightData updated =" << flight_data_ << std::endl;

        Sleep(15);
    }
}

void update_position_fixed_point(UE4Helper ue4_helper, std::shared_ptr<FlightData> flight_data_updated, std::shared_ptr<std::mutex> mtx_ptr)
{
    while (1) {
        std::lock_guard<std::mutex> guard(*mtx_ptr);
        flight_data_updated->position[0] = 200.0f; //180
        flight_data_updated->position[1] = 120.0f; //100
        flight_data_updated->position[2] = -10.0f;
        flight_data_updated->attitude[0] = -10.0f / 57.3f; //-0.3
        flight_data_updated->attitude[1] = 0.0f;
        flight_data_updated->attitude[2] = 3.8f; //3.6
        flight_data_updated->orientation = msr::airlib::VectorMath::toQuaternion(flight_data_updated->attitude[0], flight_data_updated->attitude[1], flight_data_updated->attitude[2]);
        ue4_helper.UpdatePosition(*flight_data_updated); //测试模式下不需要旋转
        //std::cout << "FlightData updated =" << *flight_data << std::endl;
    }
}

void send_image(Network network, UE4Helper ue4_helper, std::shared_ptr<FlightData> flight_data_updated)
{
    int image_index = 0;
    while (1) {
        //std::cout << "FlightInfo = " << *flight_data_updated << std::endl;
        CameraData camera_data = ue4_helper.getCameraInfo("ExternalCamera");
        //std::cout << "CameraInfo = " << camera_data << std::endl;
        UE4Helper::ImageCaptureResult image_capture_result = ue4_helper.CaptureGroundCameraImage(UE4Helper::CameraMode::ground_mode);
        std::vector<uchar> encoded_image = ue4_helper.ImageEncoder(image_capture_result);
        if (!encoded_image.empty()) {
            network.TCPSend("192.168.5.230", 17580, encoded_image, image_index, *flight_data_updated, camera_data);
            image_index++;
        }
    }
}

void save_image(Network network, UE4Helper ue4_helper, AnchorPoint anchor_point, std::shared_ptr<FlightData> flight_data_updated)
{
    cv::Mat left_wing_obj = anchor_point.TranslationMatrix(-0.350f, -1.450f, -0.027f);
    cv::Mat right_wing_obj = anchor_point.TranslationMatrix(-0.350f, 1.450f, -0.027f);
    cv::Mat left_tail_obj = anchor_point.TranslationMatrix(-1.211f, -0.3785f, -0.3315f);
    cv::Mat right_tail_obj = anchor_point.TranslationMatrix(-1.211f, 0.3785f, -0.3315f);
    cv::Mat mid_foot_obj = anchor_point.TranslationMatrix(0.709f, 0.0f, 0.330f);
    int image_index = 0;
    cv::Mat PointUV;
    cv::Mat Point3D_c;
    while (1) {
        CameraData camera_data_this = ue4_helper.getCameraInfo("ExternalCamera");
        anchor_point.CalculateAnchorPoint(*flight_data_updated, camera_data_this, PointUV, Point3D_c, left_wing_obj);
        AirSimTools::coutmat(PointUV, "left_wing_uv");
        anchor_point.CalculateAnchorPoint(*flight_data_updated, camera_data_this, PointUV, Point3D_c, right_wing_obj);
        AirSimTools::coutmat(PointUV, "right_wing_uv");
        anchor_point.CalculateAnchorPoint(*flight_data_updated, camera_data_this, PointUV, Point3D_c, left_tail_obj);
        AirSimTools::coutmat(PointUV, "left_tail_uv");
        anchor_point.CalculateAnchorPoint(*flight_data_updated, camera_data_this, PointUV, Point3D_c, right_tail_obj);
        AirSimTools::coutmat(PointUV, "right_tail_uv");
        anchor_point.CalculateAnchorPoint(*flight_data_updated, camera_data_this, PointUV, Point3D_c, mid_foot_obj);
        AirSimTools::coutmat(PointUV, "mid_foot_uv");
        UE4Helper::ImageCaptureResult image_capture_result = ue4_helper.CaptureGroundCameraImage(UE4Helper::CameraMode::ground_mode);
        int Width = image_capture_result.response.at(0).width;
        int Height = image_capture_result.response.at(0).height;
        //std::vector<uchar> encoded_image(Width * Height * 3);
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
        //cv::imshow("pic-origin", cv_image);
        //cv::waitKey(1);
        std::string image_file_name = Utils::stringf("E:/ProgramCache/AirSimDataOutput/%d.jpg", image_index);
        //msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get(); //初始化clock
        //auto start_nanos = clock->nowNanos(); //开始计时
        vector<int> encode_param = vector<int>(2);
        encode_param[0] = CV_IMWRITE_JPEG_QUALITY;
        encode_param[1] = 95;
        bool result = cv::imwrite(image_file_name, cv_image, encode_param);
        //double render_time_temp = clock->elapsedSince(start_nanos); //停止计时
        if (!result) {
            std::cout << "image save failed!" << image_file_name << std::endl;
        }
        else {
            std::cout << "image saved: " << image_file_name << std::endl;
            //std::cout << "render time: " << image_capture_result.render_time * 1E3f << " ms" << std::endl;
            //std::cout << "write time: " << render_time_temp * 1E3f << " ms" << std::endl;
        }
        image_index++;
        //Sleep(15);
    }
}

int main()
{
    Network network;
    UE4Helper ue4_helper;
    AnchorPoint anchor_point;
    std::shared_ptr<FlightData> flight_data = std::make_shared<FlightData>();
    std::shared_ptr<FlightData> flight_data_updated = std::make_shared<FlightData>();
    std::shared_ptr<std::mutex> mtx_ptr = std::make_shared<std::mutex>();

    const int mode = 4; // 1 matlab数据模式 2 本地位姿测试模式 3 X-Plane数据模式 4 固定点测试模式
    switch (mode) {
    case 1: {
        std::thread thread_matlab_flight_data_mode(matlab_flight_data_mode, network, flight_data, mtx_ptr);
        thread_matlab_flight_data_mode.detach();
        std::thread thread_updata_position(update_position, ue4_helper, flight_data, mtx_ptr);
        thread_updata_position.detach();
        std::thread thread_send_image(send_image, network, ue4_helper, flight_data_updated);
        thread_send_image.detach();
    } break;
    case 2: {
        std::thread thread_update_position_local_mode(update_position_local_mode, ue4_helper, flight_data_updated, mtx_ptr);
        thread_update_position_local_mode.detach();
        std::thread thread_save_image(save_image, network, ue4_helper, anchor_point, flight_data_updated);
        thread_save_image.detach();
    } break;
    case 3: {
        std::thread thread_xplane_flight_data_mode(xplane_flight_data_mode, ue4_helper, network, flight_data, flight_data_updated, mtx_ptr);
        thread_xplane_flight_data_mode.detach();
        std::thread thread_send_image(send_image, network, ue4_helper, flight_data_updated);
        thread_send_image.detach();
    } break;
    case 4: {
        std::thread thread_update_position_fixed_point(update_position_fixed_point, ue4_helper, flight_data_updated, mtx_ptr);
        thread_update_position_fixed_point.detach();
        //std::thread thread_send_image(send_image, network, ue4_helper, flight_data_updated);
        //thread_send_image.detach();
        std::thread thread_save_image(save_image, network, ue4_helper, anchor_point, flight_data_updated);
        thread_save_image.detach();
    }
    default:
        break;
    }

    while (true) {
        Sleep(1000);
    }
    return 0;
}
