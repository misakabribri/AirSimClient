#include "include/RpcClientGetter.h"

namespace AirSimTools
{
std::unique_ptr<msr::airlib::MultirotorRpcLibClient> RpcClientGetter::client_ptr = nullptr;
std::mutex RpcClientGetter::mtx;

void RpcClientGetter::Connect()
{
    client_ptr->confirmConnection(); //尝试连接
    client_ptr->reset(); //服务端初始化
}

msr::airlib::MultirotorRpcLibClient& RpcClientGetter::Get()
{
    std::lock_guard<std::mutex> guard(mtx);
    if (client_ptr == nullptr) {
        client_ptr = std::make_unique<msr::airlib::MultirotorRpcLibClient>();
        Connect();
    } //利用加锁，多线程下确保client_ptr仅被初始化一次
    if (client_ptr->getConnectionState() != RpcLibClientBase::ConnectionState::Connected) {
        Connect();
    } //断线重连
    return *client_ptr;
}
}