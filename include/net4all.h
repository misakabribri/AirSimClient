#ifndef _Net4all_Include_
#define _Net4all_Include_

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>

#if defined(_MSC_VER) || defined(__BORLANDC__)
#include <winsock.h>
#ifndef WINSOCK_VERSION
#define  WINSOCK_VERSION	0x0002
#endif
#define  SIGPIPE		SIGINT
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#define  INVALID_SOCKET     -1
#define  SOCKET_ERROR       -1
#define  closesocket(s)     close(s)
typedef  int   SOCKET;
#endif

///////////////////////////////////////////////////////

// Prototype declearation for socket utilties

// Version:  Release 1.0

// first create: 1999-10-28

// Last modify:  2000-08-12
// Last modify:  2002-05-20
// Last modify:  2004-03-12
// Last modify:  2006-06-29

///////////////////////////////////////////////////////

/** SOCKET start-up & clean-up(for Winsock)
 */
extern int   socketStartUp();
extern void  socketCleanUp();
extern unsigned long getHostAddress(int n);

class Socket_UDP_R
{
public:
	SOCKET          s;                 // 本节点的SOCKET资源
	sockaddr_in     stIt;              // 实际映射的网络地址
	unsigned short  port;              // 实际绑定的接收端口

	Socket_UDP_R(unsigned short p, char* local = NULL);   // 报文接收实体，指定端口，指定网卡（通过IP）
	Socket_UDP_R();                                     // 报文接收实体，动态分配端口
	Socket_UDP_R(char* new_version, int n);             // 报文接收实体，动态分配端口（指定网卡）
	Socket_UDP_R(unsigned short p, int n, int);         // 广播报文接收，指定端口（指定网卡）

	~Socket_UDP_R();

	int recv_data(char* data, int length);
};

class Socket_UDP_S
{
public:
	SOCKET          s;                  // 本节点的SOCKET资源
	sockaddr_in     stIt;               // 目的地址及端口信息

	Socket_UDP_S(unsigned short port, char* local = NULL);         // 报文发送，广播方式，指定网卡（通过IP）
	Socket_UDP_S(char* dest, unsigned short port, char* local);  // 报文发送，点对点方式，指定网卡（通过IP）
	Socket_UDP_S(char* host, unsigned short port);               // 报文发送，点对点方式，不限定网卡
	~Socket_UDP_S();

	void set_dest(u_long host_addr, unsigned short port); // 设置目的地址及端口信息
	int send_data(char* data, int length);
	int init_socket_broadcast(unsigned short port, char* local = NULL);
};

class Socket_TCP_CS
{
	void init_server(unsigned short p, int n);     // 初始化服务器端（指定第n个网卡）

public:
	SOCKET          sSock;              // 服务器端SOCKET，对客户端实体无效
	SOCKET          cSock;              // 客户端SOCKET，用于接收和发送
	sockaddr_in     stIt;               // 实际映射的网络地址
	unsigned short  port;               // 实际分配的协议端口号
	char  m_HOST_IP[16];

	Socket_TCP_CS(const char* host, unsigned short p);   // 初始化客户端，按指定的服务器地址和端口

	Socket_TCP_CS();                               // 初始化服务器端，自动分配端口
	Socket_TCP_CS(int* new_version, int n);       // 初始化服务器端，自动分配端口（指定第n个网卡）
	Socket_TCP_CS(unsigned short p, int n = -1);     // 初始化服务器端，指定协议端口（指定第n个网卡）

	long accept_connection();                      // 服务器端函数，接受客户连接
	void close_connection();                       // 服务器端函数，关闭客户连接

	~Socket_TCP_CS();

	bool connect_to_server();
	int recv_data(char* buff, int length);         // 单次接收指定长度的数据，返回实际接收的长度
	int send_data(char* data, int length);         // 单次发送指定长度的数据，返回实际发送的长度
	int recv_data_all(char* buff, int length);     // 循环接收指定长度的数据，返回实际接收的长度
	int send_data_all(char* data, int length);     // 循环发送指定长度的数据，返回实际发送的长度
};

/** 返回本地主机IP（不要释放返回的字符串）
 */
extern char* GetLocalIP(int n = 0);

/** 多播传输
 */
class Socket_MC_S
{
public:
	SOCKET          s;                  // 网络端口套接字
	sockaddr_in     stIt;               // 地址信息
	unsigned short  port;               // 协议端口号

	Socket_MC_S();
	~Socket_MC_S();

	int init_socket_multicast(char* multicat_ip, unsigned short p, int n);

	int send_data(char* data, int length);
	int recv_data(char* data, int length);
};

#endif
