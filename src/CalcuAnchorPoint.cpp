#include "include/CalcuAnchorPoint.h"
#include <string>

namespace AirSimTools
{
void coutmat(cv::Mat my_mat, std::string my_mat_name)
{
    int row = my_mat.rows;
    int col = my_mat.cols;
    std::cout << my_mat_name << " = " << std::endl
              << "[";
    for (int i = 0; i < row; i++) {
        if (i != 0)
            std::cout << std::endl;
        for (int j = 0; j < col; j++)
            if ((i == row - 1) && (j == col - 1))
                std::cout << my_mat.at<double>(i, j) << "]" << std::endl;
            else
                std::cout << my_mat.at<double>(i, j) << ", ";
    }
    //std::cout << "]" << std::endl;
}

cv::Mat CalcuAnchorPoint::IntrinsicsMatrix(float virtualFocal, float imgWidth, float imgHeight)
{
    double matrix_[3][3] = {
        { virtualFocal, 0.0, (imgWidth / 2.0) },
        { 0.0, virtualFocal, (imgHeight / 2.0) },
        { 0.0, 0.0, 1.0 }
    };
    cv::Mat intrinsics_matrix = (cv::Mat_<double>(3, 3) << matrix_[0][0], matrix_[0][1], matrix_[0][2], matrix_[1][0], matrix_[1][1], matrix_[1][2], matrix_[2][0], matrix_[2][1], matrix_[2][2]);
    intrinsics_matrix.convertTo(intrinsics_matrix, CV_64F);
    return intrinsics_matrix;
};

cv::Mat CalcuAnchorPoint::RotationMatrix(float q_x, float q_y, float q_z, float q_w)
{
    (double)q_x;
    (double)q_y;
    (double)q_z;
    (double)q_w;
    double s = q_x * q_x + q_y * q_y + q_z * q_z + q_w * q_w;
    if (s != 0) {
        s = 2.0 / s; //防止为0
    }
    double matrix_[3][3] = {
        { 1.0 - s * (q_y * q_y + q_z * q_z), s * (q_x * q_y - q_w * q_z), s * (q_x * q_z + q_w * q_y) },
        { s * (q_x * q_y + q_w * q_z), 1.0 - s * (q_x * q_x + q_z * q_z), s * (q_y * q_z - q_w * q_x) },
        { s * (q_x * q_z - q_w * q_y), s * (q_y * q_z + q_w * q_x), 1.0 - s * (q_x * q_x + q_y * q_y) }
    };
    cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) << matrix_[0][0], matrix_[0][1], matrix_[0][2], matrix_[1][0], matrix_[1][1], matrix_[1][2], matrix_[2][0], matrix_[2][1], matrix_[2][2]);
    rotation_matrix.convertTo(rotation_matrix, CV_64F);
    return rotation_matrix;
}

cv::Mat CalcuAnchorPoint::TranslationMatrix(float x, float y, float z)
{
    (double)x;
    (double)y;
    (double)z;
    cv::Mat translation_matrix = (cv::Mat_<double>(3, 1) << x, y, z);
    translation_matrix.convertTo(translation_matrix, CV_64F);
    return translation_matrix;
}

void CalcuAnchorPoint::CalculateAnchorPoint(FlightData flight_data, CameraData camera_data, cv::Mat& PointUV, cv::Mat& Point3D_c, cv::Mat Point3D)
{
    //Step 1 计算obj2wrd：猫点机体系坐标到世界系坐标 obj->wrd
    float flight_x = flight_data.position[0];
    float flight_y = flight_data.position[1];
    float flight_z = flight_data.position[2];
    float flight_q_x = flight_data.orientation.x();
    float flight_q_y = flight_data.orientation.y();
    float flight_q_z = flight_data.orientation.z();
    float flight_q_w = flight_data.orientation.w();
    cv::Mat T_obj2wrd = TranslationMatrix(flight_x, flight_y, flight_z); //机体系到世界系平动向量
    //AirSimTools::coutmat(T_obj2wrd, "T_obj2wrd");
    cv::Mat R_obj2wrd = RotationMatrix(flight_q_x, flight_q_y, flight_q_z, flight_q_w); //机体系到世界系旋转矩阵

    //Step 2 计算wrd2cam：猫点机世界系坐标到相机系坐标 wrd->cam
    float camera_x = camera_data.position[0];
    float camera_y = camera_data.position[1];
    float camera_z = camera_data.position[2];
    float camera_q_x = camera_data.orientation.x();
    float camera_q_y = camera_data.orientation.y();
    float camera_q_z = camera_data.orientation.z();
    float camera_q_w = camera_data.orientation.w();
    cv::Mat T_base2wrd = TranslationMatrix(camera_x, camera_y, camera_z); //base系到世界系平动向量
    cv::Mat R_base2wrd = RotationMatrix(camera_q_x, camera_q_y, camera_q_z, camera_q_w); //base系到世界系旋转矩阵
    cv::Mat R_wrd2base = R_base2wrd.inv(); //世界系到base系旋转矩阵
    cv::Mat T_wrd2base = -R_wrd2base * T_base2wrd; //世界系到base系平动向量
    cv::Mat R_cam2base = (cv::Mat_<float>(3, 3) << 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f); //相机系到base系旋转矩阵
    R_cam2base.convertTo(R_cam2base, CV_64F);
    cv::Mat R_base2cam = R_cam2base.inv(); //base系到相机系旋转矩阵
    cv::Mat R_wrd2cam = R_base2cam * R_wrd2base; //世界系到相机系旋转矩阵
    cv::Mat T_wrd2cam = R_base2cam * T_wrd2base; //世界系到相机系平动向量

    //Step 3 计算内参矩阵：猫点机相机系坐标到UV坐标 cam->UV
    cv::Mat intrinsics_matrix = IntrinsicsMatrix();

    //Step 4 计算猫点UV
    Point3D_c = R_wrd2cam * R_obj2wrd * Point3D + R_wrd2cam * T_obj2wrd + T_wrd2cam; //Point3D_c=[X_c,Y_c,Z_c]'
    cv::Mat Point2D_z = intrinsics_matrix * Point3D_c; //Point2D_z=[U*Z_c,V*Z_c,Z_c]'
    if (Point3D_c.at<double>(2, 0) == 0.0f)
        PointUV = (cv::Mat_<double>(2, 1) << 0.0f, 0.0f);
    else
        PointUV = (cv::Mat_<double>(1, 2) << Point2D_z.at<double>(0, 0) / Point3D_c.at<double>(2, 0), Point2D_z.at<double>(0, 1) / Point3D_c.at<double>(2, 0));
}
}
