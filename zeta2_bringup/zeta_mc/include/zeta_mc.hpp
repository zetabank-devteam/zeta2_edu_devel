#ifndef _ZETA_MC_HPP
#define _ZETA_MC_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <serial/serial.h>


typedef unsigned char 		BYTE;
typedef unsigned short 		WORD;
typedef unsigned int 		DWORD;

#define MAX_PACKET_SIZE     30
#define MAX_RECVBUFF_SIZE	512

#define ON					1
#define OFF					0

#define ENABLE            	1
#define DISABLE           	0
#define FAIL              	0	
#define SUCCESS           	1

#define LTCTS				0xA0

#define VEL_CONTROL			0x10
#define GET_POSVEL			0x20
#define GET_FW_VER			0x30

#define FW_VERSION			"MC1.1."
#define SW_VERSION			"MS2.0."


class Zeta_MC : public rclcpp::Node
{

public:	
    Zeta_MC(const std::string& node_name);
	~Zeta_MC();

    void Init(void);
	
	int InitSerial(void);
	void recvThread_Run(void);
	int ParseReceiveData(void);
	void ClearRecvBuff(void);
	void SetZeroMem(BYTE buff[], int num);
	void RecvDatafromMC(void);
	BYTE RecvDatatoBuff(BYTE buff[], int num);
	BYTE isBMRun(void) { if(b_BMRunOK) return ON; else return OFF;};
	void resetBMRunFlag(void) { b_BMRunOK = OFF; };
	void MCSerialInputCallback(const std_msgs::msg::UInt8MultiArray& msg);
	
	serial::Serial mc_serial;

    // ROS 2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr mc_serialout_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fw_version_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sw_version_pub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr mc_serialin_sub;

	
	int nBaudrate;
	BYTE sendBuf[MAX_PACKET_SIZE];
	BYTE recvBuffer[MAX_RECVBUFF_SIZE];
	BYTE PacketNumber;
	bool b_serialOpenOK;
	bool b_recvOK;
	bool b_PacketOK;
	BYTE TotalRecvNum;
	BYTE MaxDataNum;
	BYTE DataNum;
	BYTE step;
	
	BYTE ChkCommErr;
	BYTE CheckSum;

	BYTE cmd;
	BYTE pid_monitoring;
	BYTE monitoring_unit;

	bool b_SerialThreadRun;

	bool b_BMRunOK;

	bool b_StartFlag;
	int rsize;
	int startcnt;

	std::string port_name;
	
	std::thread recv_thread;

    std_msgs::msg::MultiArrayDimension mc_serialout_msg_dim;
    std_msgs::msg::UInt8MultiArray mc_serialout_msgs;
    std_msgs::msg::String fw_version_msgs;
	std_msgs::msg::String sw_version_msgs;

};

#endif