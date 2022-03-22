#pragma once
#include <iostream>

namespace AirSimTools
{
/// @brief 保存X-Plane数据的结构体
struct XPlaneData
{
    float Vtrue; // 地面空速 Vtrue（mph）
    float angular_velocities[3]; // 角速度 Q(rad/s) P(rad/s) R(rad/s)
    float attitude[3]; // 欧拉角 pitch/theta(deg) roll/phi(deg) heading/psi(deg)
    float wind_angle[2]; // 气流角 alpha(deg) beta(deg)
    float BLH[3]; //纬经高 lat(deg) lon(deg) alt(ft)

    friend std::ostream& operator<<(std::ostream& os, const XPlaneData& data)
    {
        os << "BLH{" << data.BLH[0] << ',' << data.BLH[1] << ',' << data.BLH[2] << '}'
           << " PitchRollHeading{" << data.attitude[0] << ',' << data.attitude[1] << ',' << data.attitude[2] << '}';
        return os;
    }
};
}
