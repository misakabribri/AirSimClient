#pragma once
#include "include/CameraData.h"
#include "include/FlightData.h"
#include "include/XPlaneData.h"
#include <Winsock2.h>
#include <functional>
#include <opencv/cv.h>
//#include <opencv/cxcore.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace AirSimTools
{
/// @brief RGB三个通道，最大分辨率1920*1080，为了不改变通信协议，本变量没有放在TCPImageData类中
constexpr int TCP_IMAGE_MAX_SIZE = 1920 * 1080 * 3;
/// @brief 数据包的大小,为了不改变通信协议，本变量没有放在SplitBuffer类中
constexpr int SPLIT_BUFFER = 5000;

/// @brief 通信分包
struct SplitBuffer
{
    /// @brief 图片索引
    int split_buffer_image_index;
    /// @brief 单个数据包
    char single_buffer[SPLIT_BUFFER];
};

/// @brief 图片数据缓存
struct TCPImageData
{
    /// @brief 图片数据帧头
    char tcp_data_header[128];
    /// @brief 图片数据长度
    int tcp_image_size;
    /// @brief 图片索引
    int tcp_imag_index = 0;
    int time_stamp_us;
    int time_stamp_s;
    /// @brief 飞行器目标的位姿
    float flight_data[7];
    /// @brief 相机的位姿
    float camera_data[7];
    /// @brief 图片数据容器
    unsigned char tcp_image_rgb_data[TCP_IMAGE_MAX_SIZE];
};

/// @brief 用于通信的工具人
class Network
{
public:
    SOCKET UDPSrvSock;

    /// @brief 初始化UDP，创建套接字，绑定端口
    /// @param ip_addr 需要绑定的IP地址
    /// @param port 需要绑定的端口
    void UDPInitial(std::string ip_addr, u_short port);

    /// @brief 使用UDP接收数据的函数
    /// @param ip_addr 数据来源端的IP地址
    /// @param port 数据来源端的端口
    /// @param recv_buff 接收UDP数据的缓存容器
    /// @param sender_addr 用于确认发送端的ip，缺省即可（Structure used by kernel to store most addresses）
    /// @todo 现在建立链接与发送数据未分开
    void UDPReceive(std::string ip_addr, u_short port, std::vector<char>& recv_buff, SOCKADDR& sender_addr);

    /// @brief 使用UDP接收数据的函数（重载）
    /// @param ip_addr 数据来源端的IP地址
    /// @param port 数据来源端的端口
    /// @param recv_buff 接收UDP数据的缓存容器
    void UDPReceive(std::string ip_addr, u_short port, std::vector<char>& recv_buff)
    {
        SOCKADDR sender_addr;
        UDPReceive(ip_addr, port, recv_buff, sender_addr);
    }

    /// @brief 使用UDP发送数据的函数
    /// @param ip_addr 目标IP
    /// @param port 目标端口
    /// @param recv_buff
    void UDPSend(std::string ip_addr, u_short port, std::vector<char>& recv_buff, SOCKADDR& sender_addr);

    /// @brief 使用UDP发送数据的函数（重载）
    /// @param ip_addr 目标IP
    /// @param port 目标端口
    /// @param recv_buff
    void UDPSend(std::string ip_addr, u_short port, std::vector<char>& recv_buff)
    {
        SOCKADDR sender_addr;
        UDPSend(ip_addr, port, recv_buff, sender_addr);
    }

    /// @brief 提供图源数据的TCP服务端
    /// @param ip_addr 客户端的IP地址
    /// @param port 客户端的端口
    /// @param tcp_send_data 需要发送的数据
    /// @param image_index 图片索引
    /// @param flight_data 飞行器位姿数据
    /// @param camera_data 相机外参数据
    /// @todo 现在TCP建立链接与发送数据未分开
    void Network::TCPSend(std::string ip_addr, u_short port, std::vector<uchar>& tcp_send_data, int image_index, FlightData flight_data, CameraData camera_data);

    /// @brief UDP通信解帧函数（来自Matlab的UDP模块）
    /// @param recv_buff 接收到的UDP缓存数据
    /// @todo 优化：功能增加
    FlightData DecodeByteMATLAB(std::vector<char>& recv_buff);

    /// @brief UDP通信解帧函数（来自X-Plane）
    /// @param recv_buff 接收到的UDP缓存数据
    /// @return XPlaneData结构体
    XPlaneData Network::DecodeByteXPlane(std::vector<char>& recv_buff);
};
}
