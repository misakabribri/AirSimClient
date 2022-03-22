/***********************************************************
  BLH -> ENU 変換
  : WGS84 の緯度(Beta)／経度(Lambda)／楕円体高(Height)を
    ENU (East/North/Up; 地平) 座標に変換する。
    * 途中、 ECEF（Earth Centered Earth Fixed; 地球中心・地
      球固定直交座標系）座標への変換を経由。
    DATE        AUTHOR       VERSION
    2021.05.06  mk-mode.com  1.00 新規作成
  Copyright(C) 2021 mk-mode.com All Rights Reserved.
  ----------------------------------------------------------
  引数 : B_0 L_0 H_0 B_1 L_1 H_1
         * B_0, L_0, H_0: 基準の BLH(WGS84) 座標
         * B_1, L_1, H_1: 対象の BLH(WGS84) 座標
  ----------------------------------------------------------
  $ g++102 -std=c++17 -Wall -O2 --pedantic-errors -o blh2enu blh2enu.cpp
***********************************************************/

#include "include/LBH2NED.h"
#include <cmath>
#include <cstdlib> // for EXIT_XXXX
#include <iomanip>
#include <iostream>
#include <vector>

namespace AirSimTools
{
std::vector<std::vector<double>> BLH2XYZ::mat_x(double ang)
{
    double a;
    double c;
    double s;
    std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

    try {
        a = ang * kPi180;
        c = cos(a);
        s = sin(a);
        mat[0][0] = 1.0;
        mat[1][1] = c;
        mat[1][2] = s;
        mat[2][1] = -s;
        mat[2][2] = c;
    }
    catch (...) {
        throw;
    }

    return mat;
}

std::vector<std::vector<double>> BLH2XYZ::mat_y(double ang)
{
    double a;
    double c;
    double s;
    std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

    try {
        a = ang * kPi180;
        c = cos(a);
        s = sin(a);
        mat[0][0] = c;
        mat[0][2] = -s;
        mat[1][1] = 1.0;
        mat[2][0] = s;
        mat[2][2] = c;
    }
    catch (...) {
        throw;
    }

    return mat;
}

std::vector<std::vector<double>> BLH2XYZ::mat_z(double ang)
{
    double a;
    double c;
    double s;
    std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

    try {
        a = ang * kPi180;
        c = cos(a);
        s = sin(a);
        mat[0][0] = c;
        mat[0][1] = s;
        mat[1][0] = -s;
        mat[1][1] = c;
        mat[2][2] = 1.0;
    }
    catch (...) {
        throw;
    }

    return mat;
}

std::vector<std::vector<double>> BLH2XYZ::mul_mat(std::vector<std::vector<double>> mat_a, std::vector<std::vector<double>> mat_b)
{
    unsigned int i;
    unsigned int j;
    unsigned int k;
    std::vector<std::vector<double>> mat(3, std::vector<double>(3, 0.0));

    try {
        for (i = 0; i < 3; ++i) {
            for (j = 0; j < 3; ++j) {
                for (k = 0; k < 3; ++k) {
                    mat[i][j] += mat_a[i][k] * mat_b[k][j];
                }
            }
        }
    }
    catch (...) {
        throw;
    }

    return mat;
}

BLH2XYZ::CoordE BLH2XYZ::rotate(std::vector<std::vector<double>> mat_r, CoordX pt_0)
{
    CoordE pt;

    try {
        pt.e = mat_r[0][0] * pt_0.x + mat_r[0][1] * pt_0.y + mat_r[0][2] * pt_0.z;
        pt.n = mat_r[1][0] * pt_0.x + mat_r[1][1] * pt_0.y + mat_r[1][2] * pt_0.z;
        pt.u = mat_r[2][0] * pt_0.x + mat_r[2][1] * pt_0.y + mat_r[2][2] * pt_0.z;
    }
    catch (...) {
        throw;
    }

    return pt;
}

double BLH2XYZ::n(double x)
{
    double res;

    try {
        res = kA / sqrt(1.0 - kE2 * pow(sin(x * kPi180), 2));
    }
    catch (...) {
        throw;
    }

    return res;
}

BLH2XYZ::CoordX BLH2XYZ::blh2ecef(CoordB c_src)
{
    CoordX ecef;

    try {
        ecef.x = (n(c_src.b) + c_src.h) * cos(c_src.b * kPi180) * cos(c_src.l * kPi180);
        ecef.y = (n(c_src.b) + c_src.h) * cos(c_src.b * kPi180) * sin(c_src.l * kPi180);
        ecef.z = (n(c_src.b) * (1.0 - kE2) + c_src.h) * sin(c_src.b * kPi180);
    }
    catch (...) {
        throw;
    }

    return ecef;
}

BLH2XYZ::CoordE BLH2XYZ::blh2enu(CoordB c_0, CoordB c_1)
{
    CoordX ecef_x_0;
    CoordX ecef_x_1;
    CoordX ecef_x;
    std::vector<std::vector<double>> mat_0(3, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> mat_1(3, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> mat_2(3, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> mat;
    CoordE enu;

    try {
        // BLH -> ECEF
        ecef_x_0 = blh2ecef(c_0);
        ecef_x_1 = blh2ecef(c_1);
        ecef_x = { ecef_x_1.x - ecef_x_0.x,
                   ecef_x_1.y - ecef_x_0.y,
                   ecef_x_1.z - ecef_x_0.z };
        // 回転行列生成
        mat_0 = mat_z(90.0);
        mat_1 = mat_y(90.0 - c_0.b);
        mat_2 = mat_z(c_0.l);
        mat = mul_mat(mul_mat(mat_0, mat_1), mat_2);
        enu = rotate(mat, ecef_x);
    }
    catch (...) {
        throw;
    }

    return enu;
}

FlightData BLH2XYZ::BLH2NED(XPlaneData XPlaneData_, float L0, float B0, float H0)
{
    BLH2XYZ BLH2XYZ_; //实例化一个坐标转换工具人
    FlightData FlightData_;
    CoordB blh_0; // 変換前の座標（基準）
    CoordB blh_1; // 変換前の座標（対象）
    CoordE enu; // 変換後の座標
    blh_0.b = B0;
    blh_0.l = L0;
    blh_0.h = H0;
    blh_1.b = XPlaneData_.BLH[0];
    blh_1.l = XPlaneData_.BLH[1];
    blh_1.h = XPlaneData_.BLH[2] * 0.3048;

    enu = BLH2XYZ_.blh2enu(blh_0, blh_1); // BLH -> ENU

    FlightData_.position[0] = (float)enu.n;
    FlightData_.position[1] = (float)enu.e;
    FlightData_.position[2] = (float)-enu.u;
    FlightData_.attitude[0] = (float)XPlaneData_.attitude[0] / 57.3f; //pitch(rad)
    FlightData_.attitude[1] = (float)XPlaneData_.attitude[1] / 57.3f; //roll(rad)
    FlightData_.attitude[2] = (float)XPlaneData_.attitude[2] / 57.3f; //heading(rad)
    return FlightData_;
}
} // namespace CoordinateTools