#include "include/Network.h"
#include "include/UE4Helper.h"
#include "include/net4all.h"
#include <iostream>
#include <mutex>
#include <thread>

#pragma warning(disable : 4996)
#pragma warning(disable : 4189)
#pragma warning(disable : 4267)
#pragma comment(lib, "ws2_32.lib")

namespace AirSimTools
{
void Network::UDPInitial(std::string ip_addr, u_short port)
{
    WORD wVersionRequested; //套接字库版本号
    WSADATA wsaData;
    int err;
    wVersionRequested = MAKEWORD(2, 2); //定义套接字的版本号
    err = WSAStartup(wVersionRequested, &wsaData); //创建套接字及失败处理
    int check_flag = 1;
    if (err != 0) {
        check_flag = 0;
    }
    else if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
        WSACleanup();
        check_flag = 0;
    }
    UDPSrvSock = socket(AF_INET, SOCK_DGRAM, 0); //创建套接字
    SOCKADDR_IN SrvAddr;
    SrvAddr.sin_addr.S_un.S_addr =
        inet_addr(ip_addr.c_str()); //数据发送端IP地址
    SrvAddr.sin_family = AF_INET; //服务端地址族
    SrvAddr.sin_port = htons(port); //数据发送端口号
    bind(UDPSrvSock, (SOCKADDR*)&SrvAddr, sizeof(SOCKADDR)); //绑定IP地址及端口
    int len = sizeof(SOCKADDR);
}

void Network::UDPReceive(std::string ip_addr, u_short port, std::vector<char>& recv_buff, SOCKADDR& sender_addr)
{
    int len = sizeof(SOCKADDR);
    UDPInitial(ip_addr, port);
    if (recvfrom(UDPSrvSock, recv_buff.data(), (int)recv_buff.size(), 0, &sender_addr, &len) == SOCKET_ERROR) //接收UDP数据
    {
        wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
    } // 返回值，错误检查

    closesocket(UDPSrvSock); //关闭套接字
    WSACleanup();
}

void Network::UDPSend(std::string ip_addr, u_short port, std::vector<char>& recv_buff, SOCKADDR& sender_addr)
{
    WORD wVersionRequested; //套接字库版本号
    WSADATA wsaData;
    int err;
    wVersionRequested = MAKEWORD(2, 2); //定义套接字的版本号
    err = WSAStartup(wVersionRequested, &wsaData); //创建套接字
    int check_flag = 1;
    if (err != 0) {
        check_flag = 0;
    }
    ///创建套接字失败处理
    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
        WSACleanup();
        check_flag = 0;
    }
    SOCKET ClistSock = socket(AF_INET, SOCK_DGRAM, 0); //创建套接字类型
    SOCKADDR_IN SrvAddr;
    SrvAddr.sin_family = AF_INET; //选择地址族
    SrvAddr.sin_addr.S_un.S_addr = inet_addr(ip_addr.c_str());
    SrvAddr.sin_port = htons(port); //服务端的端口号
    int check = sendto(ClistSock, recv_buff.data(), (int)recv_buff.size(), 0, (SOCKADDR*)&SrvAddr, sizeof(SOCKADDR));
    closesocket(ClistSock); //关闭套接字
    WSACleanup();
}

void Network::TCPSend(std::string ip_addr, u_short port, std::vector<uchar>& tcp_send_data, int image_index, FlightData flight_data, CameraData camera_data)
{
    std::unique_ptr<Socket_TCP_CS> g_Socket(new Socket_TCP_CS(ip_addr.c_str(), port)); // 智能指针
    g_Socket->connect_to_server();
    std::unique_ptr<TCPImageData> tcp_image_data(new TCPImageData());
    for (int i = 0; i < tcp_send_data.size(); i++)
        tcp_image_data->tcp_image_rgb_data[i] = tcp_send_data[i];
    tcp_image_data->tcp_image_size = (int)tcp_send_data.size();
    tcp_image_data->tcp_imag_index = image_index;
    tcp_image_data->flight_data[0] = flight_data.position[0]; //x
    tcp_image_data->flight_data[1] = flight_data.position[1]; //y
    tcp_image_data->flight_data[2] = flight_data.position[2]; //z
    tcp_image_data->flight_data[3] = flight_data.orientation.x(); //qx
    tcp_image_data->flight_data[4] = flight_data.orientation.y(); //qy
    tcp_image_data->flight_data[5] = flight_data.orientation.z(); //qz
    tcp_image_data->flight_data[6] = flight_data.orientation.w(); //qw
    //std::cout << tcp_image_data->flight_data[3] << "," << tcp_image_data->flight_data[4] << "," << tcp_image_data->flight_data[5] << "," << tcp_image_data->flight_data[6];
    tcp_image_data->camera_data[0] = camera_data.position[0]; //x
    tcp_image_data->camera_data[1] = camera_data.position[1]; //y
    tcp_image_data->camera_data[2] = camera_data.position[2]; //z
    tcp_image_data->camera_data[3] = camera_data.orientation.x(); //qx
    tcp_image_data->camera_data[4] = camera_data.orientation.y(); //qy
    tcp_image_data->camera_data[5] = camera_data.orientation.z(); //qz
    tcp_image_data->camera_data[6] = camera_data.orientation.w(); //qw
    SplitBuffer split_buffer;
    int send_size = 0;
    int package_size =
        sizeof(TCPImageData) - TCP_IMAGE_MAX_SIZE + tcp_image_data->tcp_image_size;
    while (send_size < package_size) {
        int buffer_size = std::min(package_size - send_size, SPLIT_BUFFER);
        memcpy(split_buffer.single_buffer, (char*)(tcp_image_data.get()) + send_size, buffer_size);
        split_buffer.split_buffer_image_index = tcp_image_data->tcp_imag_index;
        int ret = g_Socket->send_data((char*)&split_buffer, sizeof(SplitBuffer));
        if (ret <= 0) {
            std::cout << "Connection lost ! Reconneting..." << std::endl;
            // Reconnecting
            bool connectting = false;
            while (!connectting) {
                connectting = g_Socket->connect_to_server();
                Sleep(1000);
            }
            std::cout << "Reconnet successed!" << std::endl;
        }
        else
            send_size += (ret - sizeof(int));
    }
}

FlightData Network::DecodeByteMATLAB(std::vector<char>& recv_buff)
{
    FlightData flight_data;
    // std::string str = recv_buff.data();
    if (recv_buff[0] == 'D' && recv_buff[1] == 'A') // 数据帧头"DATA@"校验
    {
        double* Data[6]; // 共计6个float需要读取
        int ByteCounter = 5; // 除去帧头，从第6个byte开始
        for (int N = 0; N <= 5; N++) {
            Data[N] = (double*)&recv_buff[ByteCounter];
            ByteCounter = ByteCounter + 8;
        }
        flight_data.position[0] = (float)*Data[0]; // X
        flight_data.position[1] = (float)*Data[1]; // Y
        flight_data.position[2] = (float)-*Data[2]; // Z
        flight_data.attitude[0] = (float)*Data[3]; // theta
        flight_data.attitude[1] = (float)*Data[4]; // phi
        flight_data.attitude[2] = (float)*Data[5]; // psi
    }
    return flight_data;
}

XPlaneData Network::DecodeByteXPlane(std::vector<char>& recv_buff)
{
    XPlaneData XPlaneData_;
    uint32_t* IndexID[5];
    float* XPlaneData[5][8]; // 分配空间，X-Plane中勾选了5行数据，每行1个unin32+8个float
    if (recv_buff[0] == 'D' && recv_buff[1] == 'A') // 数据帧头"DATA@"校验
    {
        int ByteCounter = 5; // 数据帧头长度
        for (int i = 0; i < 5; i++) // 共计5个数据包
        {
            for (int j = 0; j < 9; j++) // 每个数据包9个数据（1+8）
            {
                if (j < 1) {
                    IndexID[i] = (uint32_t*)&recv_buff[ByteCounter]; // 数据包ID，含义对应X-Plane的Data界面
                }
                else {
                    XPlaneData[i][j - 1] = (float*)&recv_buff[ByteCounter]; // 记录数据包数据，这里注意是j-1，因为XPlaneData数组仍然从0开始计数
                }
                ByteCounter = ByteCounter + 4;
            }
        }
    }
    XPlaneData_.Vtrue = *XPlaneData[0][7]; // Vtrue(mph)
    XPlaneData_.angular_velocities[0] = *XPlaneData[1][0]; // Q(rad/s)
    XPlaneData_.angular_velocities[1] = *XPlaneData[1][1]; // P(rad/s)
    XPlaneData_.angular_velocities[2] = *XPlaneData[1][2]; // R(rad/s)
    XPlaneData_.attitude[0] = *XPlaneData[2][0]; // pitch(deg)
    XPlaneData_.attitude[1] = *XPlaneData[2][1]; // roll(deg)
    XPlaneData_.attitude[2] = *XPlaneData[2][2]; // heading(deg)
    XPlaneData_.wind_angle[0] = *XPlaneData[3][0]; // alpha(deg)
    XPlaneData_.wind_angle[1] = *XPlaneData[3][1]; // beta(deg)
    XPlaneData_.BLH[0] = *XPlaneData[4][0]; // lat(deg)
    XPlaneData_.BLH[1] = *XPlaneData[4][1]; // lon(deg)
    XPlaneData_.BLH[2] = *XPlaneData[4][2]; // alt(ft)
    return XPlaneData_;
}
} // namespace AirSimTools
