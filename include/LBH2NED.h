#pragma once
#include "include/FlightData.h"
#include "include/XPlaneData.h"
#include <vector>

using XPlaneData = AirSimTools::XPlaneData;
/// @brief 坐标转换函数
namespace AirSimTools
{
/// @brief 将GPS纬经高转直角坐标系的工具人
class BLH2XYZ
{
public:
    // [ WGS84 座標パラメータ ]
    double kPi = atan(1.0) * 4.0;
    double kPi180 = kPi / 180;
    static constexpr double kA = 6378137.0; // a(地球楕円体長半径(赤道面平均半径))
    static constexpr double k1F = 298.257223563; // 1 / f(地球楕円体扁平率=(a - b) / a)
    static constexpr double kB = kA * (1.0 - 1.0 / k1F); // b(地球楕円体短半径)
    static constexpr double kE2 = (1.0 / k1F) * (2.0 - (1.0 / k1F)); // e^2 = 2 * f - f * f = (a^2 - b^2) / a^2
    static constexpr double kEd2 = kE2 * kA * kA / (kB * kB); // e'^2= (a^2 - b^2) / b^2

    /// @brief 纬经高坐标
    struct CoordB
    {
        double b; // B(Beta)
        double l; // L(Lambda)
        double h; // H(Height)
    };

    /// @brief 直角坐标
    struct CoordX
    {
        double x; // X
        double y; // Y
        double z; // Z
    };

    /// @brief ENU坐标
    struct CoordE
    {
        double e; // E(East)
        double n; // N(North)
        double u; // U(Up)
    };


    /// @brief x 軸を軸とした回転行列
    /// @param ang 回転量（°） (double)
    /// @return 回転行列(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_x(double ang);

    /// @brief y 軸を軸とした回転行列
    /// @param ang 回転量（°） (double)
    /// @return 回転行列(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_y(double ang);

    /// @brief z 軸を軸とした回転行列
    /// @param ang 回転量（°） (double)
    /// @return 回転行列(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_z(double ang);

    /// @brief 2行列(3x3)の積
    /// @param mat_a 元の 3x3 行列 (vector<vector<double>>)
    /// @param mat_b 元の 3x3 行列 (vector<vector<double>>)
    /// @return 計算後の 3x3 行列 (vector<vector<double>>)
    std::vector<std::vector<double>> mul_mat(std::vector<std::vector<double>> mat_a, std::vector<std::vector<double>> mat_b);

    /// @brief 座標の回転
    /// @param mat_r 3x3 回転行列 (vector<vector<double>>)
    /// @param pt_0 回転前座標 (CoordX)
    /// @return 回転後座標 (CoordE)
    CoordE rotate(std::vector<std::vector<double>> mat_r, CoordX pt_0);

    /// @brief 関数 N
    /// @param x X (double)
    /// @return 計算結果 (double)
    double n(double x);

    /// @brief BLH -> ECEF
    /// @param c_src BLH  座標 (CoordB)
    /// @return ECEF 座標 (CoordX)
    CoordX blh2ecef(CoordB c_src);

    /// @brief BLH -> ENU
    /// @param c_0 BLH 座標（基準） (CoordB)
    /// @param c_1 BLH 座標（対象） (CoordB)
    /// @return ENU 座標 (CoordE)
    CoordE blh2enu(CoordB c_0, CoordB c_1);
    /// @brief 经纬高BLH转北东下坐标系
    /// @param XPlaneData_ X-Plane数据结构体
    /// @param L0 原点经度
    /// @param B0 原点纬度
    /// @param H0 原点高度
    /// @return 飞行器位姿数据结构体
    FlightData BLH2NED(XPlaneData XPlaneData_, float L0 = 112.9338225f, float B0 = 28.29625f, float H0 = -0.20814f * 0.3048f);
};
}