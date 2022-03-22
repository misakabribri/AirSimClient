#pragma once
#include <iostream>

namespace AirSimTools
{
/// @brief ����X-Plane���ݵĽṹ��
struct XPlaneData
{
    float Vtrue; // ������� Vtrue��mph��
    float angular_velocities[3]; // ���ٶ� Q(rad/s) P(rad/s) R(rad/s)
    float attitude[3]; // ŷ���� pitch/theta(deg) roll/phi(deg) heading/psi(deg)
    float wind_angle[2]; // ������ alpha(deg) beta(deg)
    float BLH[3]; //γ���� lat(deg) lon(deg) alt(ft)

    friend std::ostream& operator<<(std::ostream& os, const XPlaneData& data)
    {
        os << "BLH{" << data.BLH[0] << ',' << data.BLH[1] << ',' << data.BLH[2] << '}'
           << " PitchRollHeading{" << data.attitude[0] << ',' << data.attitude[1] << ',' << data.attitude[2] << '}';
        return os;
    }
};
}
