#pragma once
#include "common/Common.hpp"
#include "include/CameraData.h"
#include "include/FlightData.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma warning(disable : 4003)

namespace AirSimTools
{
/// @brief
/// @param my_mat 被输出mat
/// @param my_mat_name 被输出mat的名称
/// @note opencv版本过于老旧，在VC++14下无法正确重载 cout<<
void coutmat(cv::Mat my_mat, std::string my_mat_name);

/// @brief 计算猫点的工具人
class CalcuAnchorPoint
{
public:
    /// @brief 计算内参矩阵
    /// @param virtualFocal
    /// @param imgWidth
    /// @param imgHeight
    /// @return intrinsics_matrix
    cv::Mat IntrinsicsMatrix(float virtualFocal = 24939.6f, float imgWidth = 1280.0f, float imgHeight = 720.0f);

    /// @brief 计算旋转矩阵
    /// @param q_x 四元数x
    /// @param q_y 四元数y
    /// @param q_z 四元数z
    /// @param q_w 四元数w
    /// @return rotation_matrix
    cv::Mat RotationMatrix(float q_x, float q_y, float q_z, float q_w);

    /// @brief 计算平动矩阵
    /// @param x x坐标
    /// @param y y坐标
    /// @param z z坐标
    /// @return translation_matrix
    cv::Mat TranslationMatrix(float x, float y, float z);

    /// @brief 计算锚点UV
    /// @param flight_data 飞行器在世界系的位姿
    /// @param camera_data 相机在世界系的位姿
    /// @param PointUV 引用锚点的UV（通过外部传入）
    /// @param Point3D_c 引用锚点在相机系的坐标（通过外部传入）
    /// @param Point3D 锚点在机体系的坐标
    void CalculateAnchorPoint(FlightData flight_data, CameraData camera_data, cv::Mat& PointUV, cv::Mat& Point3D_c, cv::Mat Point3D);
};
}