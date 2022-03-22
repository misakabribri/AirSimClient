#pragma once
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include <mutex>

namespace AirSimTools
{
/// @brief ����ģʽ������ʼ��һ��
class RpcClientGetter
{
private:
    /// @brief ����ָ�����ԭ��ָ��new
    static std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_ptr;
    /// @brief ������ģʽ�Ķ��߳�����
    static std::mutex mtx;

public:
    /// @brief ����clientʵ��
    static msr::airlib::MultirotorRpcLibClient& Get();

private:
    /// @brief ����ģʽ��ֹʵ����
    RpcClientGetter() = default;
    /// @brief ����rpc������UE4
    static void Connect();
};
}