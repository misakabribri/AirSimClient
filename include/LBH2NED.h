#pragma once
#include "include/FlightData.h"
#include "include/XPlaneData.h"
#include <vector>

using XPlaneData = AirSimTools::XPlaneData;
/// @brief ����ת������
namespace AirSimTools
{
/// @brief ��GPSγ����תֱ������ϵ�Ĺ�����
class BLH2XYZ
{
public:
    // [ WGS84 ���˥ѥ��`�� ]
    double kPi = atan(1.0) * 4.0;
    double kPi180 = kPi / 180;
    static constexpr double kA = 6378137.0; // a(����������L�뾶(�����ƽ���뾶))
    static constexpr double k1F = 298.257223563; // 1 / f(����������ƽ��=(a - b) / a)
    static constexpr double kB = kA * (1.0 - 1.0 / k1F); // b(���������̰뾶)
    static constexpr double kE2 = (1.0 / k1F) * (2.0 - (1.0 / k1F)); // e^2 = 2 * f - f * f = (a^2 - b^2) / a^2
    static constexpr double kEd2 = kE2 * kA * kA / (kB * kB); // e'^2= (a^2 - b^2) / b^2

    /// @brief γ��������
    struct CoordB
    {
        double b; // B(Beta)
        double l; // L(Lambda)
        double h; // H(Height)
    };

    /// @brief ֱ������
    struct CoordX
    {
        double x; // X
        double y; // Y
        double z; // Z
    };

    /// @brief ENU����
    struct CoordE
    {
        double e; // E(East)
        double n; // N(North)
        double u; // U(Up)
    };


    /// @brief x �S���S�Ȥ�����ܞ����
    /// @param ang ��ܞ�����㣩 (double)
    /// @return ��ܞ����(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_x(double ang);

    /// @brief y �S���S�Ȥ�����ܞ����
    /// @param ang ��ܞ�����㣩 (double)
    /// @return ��ܞ����(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_y(double ang);

    /// @brief z �S���S�Ȥ�����ܞ����
    /// @param ang ��ܞ�����㣩 (double)
    /// @return ��ܞ����(3x3) (vector<vector<double>>)
    std::vector<std::vector<double>> mat_z(double ang);

    /// @brief 2����(3x3)�ηe
    /// @param mat_a Ԫ�� 3x3 ���� (vector<vector<double>>)
    /// @param mat_b Ԫ�� 3x3 ���� (vector<vector<double>>)
    /// @return Ӌ����� 3x3 ���� (vector<vector<double>>)
    std::vector<std::vector<double>> mul_mat(std::vector<std::vector<double>> mat_a, std::vector<std::vector<double>> mat_b);

    /// @brief ���ˤλ�ܞ
    /// @param mat_r 3x3 ��ܞ���� (vector<vector<double>>)
    /// @param pt_0 ��ܞǰ���� (CoordX)
    /// @return ��ܞ������ (CoordE)
    CoordE rotate(std::vector<std::vector<double>> mat_r, CoordX pt_0);

    /// @brief �v�� N
    /// @param x X (double)
    /// @return Ӌ��Y�� (double)
    double n(double x);

    /// @brief BLH -> ECEF
    /// @param c_src BLH  ���� (CoordB)
    /// @return ECEF ���� (CoordX)
    CoordX blh2ecef(CoordB c_src);

    /// @brief BLH -> ENU
    /// @param c_0 BLH ���ˣ����ʣ� (CoordB)
    /// @param c_1 BLH ���ˣ����� (CoordB)
    /// @return ENU ���� (CoordE)
    CoordE blh2enu(CoordB c_0, CoordB c_1);
    /// @brief ��γ��BLHת����������ϵ
    /// @param XPlaneData_ X-Plane���ݽṹ��
    /// @param L0 ԭ�㾭��
    /// @param B0 ԭ��γ��
    /// @param H0 ԭ��߶�
    /// @return ������λ�����ݽṹ��
    FlightData BLH2NED(XPlaneData XPlaneData_, float L0 = 112.9338225f, float B0 = 28.29625f, float H0 = -0.20814f * 0.3048f);
};
}