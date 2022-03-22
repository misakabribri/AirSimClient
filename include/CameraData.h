#pragma once
#include "common/Common.hpp"
#include <iostream>

namespace AirSimTools
{
/// @brief �������λ�ú���̬�Ľṹ��
struct CameraData
{
    float attitude[3] = { 0, 0, 0 }; // ŷ���� pitch(theta) roll(phi) heading(psi)
    float position[3] = { 0, 0, 0 }; // λ�� x y z
    msr::airlib::Quaternionr orientation;

    friend std::ostream& operator<<(std::ostream& os, const CameraData& data)
    {
        os << "p{" << data.position[0] << ',' << data.position[1] << ',' << data.position[2] << '}'
           << " a{" << data.attitude[0] << ',' << data.attitude[1] << ',' << data.attitude[2] << '}'
           << " q{" << data.orientation.x() << ',' << data.orientation.y() << ',' << data.orientation.z() << ',' << data.orientation.w() << '}';
        return os;
    }
};
}
