#include "include/RpcClientGetter.h"

namespace AirSimTools
{
std::unique_ptr<msr::airlib::MultirotorRpcLibClient> RpcClientGetter::client_ptr = nullptr;
std::mutex RpcClientGetter::mtx;

void RpcClientGetter::Connect()
{
    client_ptr->confirmConnection(); //��������
    client_ptr->reset(); //����˳�ʼ��
}

msr::airlib::MultirotorRpcLibClient& RpcClientGetter::Get()
{
    std::lock_guard<std::mutex> guard(mtx);
    if (client_ptr == nullptr) {
        client_ptr = std::make_unique<msr::airlib::MultirotorRpcLibClient>();
        Connect();
    } //���ü��������߳���ȷ��client_ptr������ʼ��һ��
    if (client_ptr->getConnectionState() != RpcLibClientBase::ConnectionState::Connected) {
        Connect();
    } //��������
    return *client_ptr;
}
}