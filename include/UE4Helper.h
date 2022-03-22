#pragma once
#include "common/Common.hpp"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/ProsumerQueue.hpp"
#include "include/CameraData.h"
#include "include/FlightData.h"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/// @brief AirSim 通信及API程序
namespace AirSimTools
{
/// @brief 调用UE4的工具人
class UE4Helper
{
private:
    typedef common_utils::FileSystem FileSystem;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
    typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

public:
    struct ImageCaptureResult
    {
        std::vector<ImageResponse> response;
        msr::airlib::TTimeDelta render_time;
        std::string storage_dir_;
        std::fstream* file_list;
        int sample;
        Vector3r position;
        Quaternionr orientation;
    };

    enum class CameraMode : int
    {
        origin_mode, //适用于原版
        ground_mode //适用于修改后AirSim版本，地面相机固定
    };

    /// @brief 校正UE4地图航向误差
    /// @param flight_data
    /// @param psi_error 单位degree，逆时针旋转为负
    /// @return 校正过的位姿数据
    FlightData AdaptPosition(const FlightData& flight_data, float pitch_modified = -4.6510f, float roll_modified = 0.0639f, float yaw_modified = -120.7494f + 90.0f - 62.6383f); //-120.75 + 90 - 62.638306
    /// @brief 更新无人机位置
    /// @param flight_data
    void UpdatePosition(const FlightData& flight_data);
    /// @brief 获取相机信息
    CameraData UE4Helper::getCameraInfo(const std::string& camera_name);
    /// @brief 获取相机图片
    ImageCaptureResult CaptureGroundCameraImage(CameraMode camera_mode);
    /// @brief 处理相机图片
    /// @param image_capture_result
    void ProcessCameraImage(ImageCaptureResult& image_capture_result);
    /// @brief 使用opencv进行图片编码
    /// @param image_capture_result
    /// @return 编码后的图片
    std::vector<uchar> ImageEncoder(ImageCaptureResult& image_capture_result);
};
}
