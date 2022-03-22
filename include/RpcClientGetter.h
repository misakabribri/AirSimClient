#pragma once
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include <mutex>

namespace AirSimTools
{
/// @brief 单例模式，仅初始化一次
class RpcClientGetter
{
private:
    /// @brief 智能指针替代原生指针new
    static std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_ptr;
    /// @brief 处理单例模式的多线程问题
    static std::mutex mtx;

public:
    /// @brief 返回client实例
    static msr::airlib::MultirotorRpcLibClient& Get();

private:
    /// @brief 单例模式禁止实例化
    RpcClientGetter() = default;
    /// @brief 尝试rpc连接至UE4
    static void Connect();
};
}