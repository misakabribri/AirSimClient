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
/// @param my_mat �����mat
/// @param my_mat_name �����mat������
/// @note opencv�汾�����Ͼɣ���VC++14���޷���ȷ���� cout<<
void coutmat(cv::Mat my_mat, std::string my_mat_name);

/// @brief ����è��Ĺ�����
class CalcuAnchorPoint
{
public:
    /// @brief �����ڲξ���
    /// @param virtualFocal
    /// @param imgWidth
    /// @param imgHeight
    /// @return intrinsics_matrix
    cv::Mat IntrinsicsMatrix(float virtualFocal = 24939.6f, float imgWidth = 1280.0f, float imgHeight = 720.0f);

    /// @brief ������ת����
    /// @param q_x ��Ԫ��x
    /// @param q_y ��Ԫ��y
    /// @param q_z ��Ԫ��z
    /// @param q_w ��Ԫ��w
    /// @return rotation_matrix
    cv::Mat RotationMatrix(float q_x, float q_y, float q_z, float q_w);

    /// @brief ����ƽ������
    /// @param x x����
    /// @param y y����
    /// @param z z����
    /// @return translation_matrix
    cv::Mat TranslationMatrix(float x, float y, float z);

    /// @brief ����ê��UV
    /// @param flight_data ������������ϵ��λ��
    /// @param camera_data ���������ϵ��λ��
    /// @param PointUV ����ê���UV��ͨ���ⲿ���룩
    /// @param Point3D_c ����ê�������ϵ�����꣨ͨ���ⲿ���룩
    /// @param Point3D ê���ڻ���ϵ������
    void CalculateAnchorPoint(FlightData flight_data, CameraData camera_data, cv::Mat& PointUV, cv::Mat& Point3D_c, cv::Mat Point3D);
};
}