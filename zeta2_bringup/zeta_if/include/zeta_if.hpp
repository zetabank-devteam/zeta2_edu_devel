#ifndef _ZETA_IF_HPP
#define _ZETA_IF_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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

#define SEND_SONAR_DATA		0x10
#define SEND_IMU_DATA		0x20
#define SEND_HWFW_VER		0x30
#define SEND_LINEDETECT		0x40

#define HW_VERSION			"MCB1.1"
#define FW_VERSION			"IF1.0."
#define SW_VERSION			"IS2.0."


class Zeta_IF : public rclcpp::Node
{
	
public:	
	Zeta_IF(const std::string& node_name);
	~Zeta_IF();

    void Init(void);
	
	int InitSerial(void);
	void recvThread_Run(void);
	int ParseReceiveData(void);
	void ClearRecvBuff(void);
	void SetZeroMem(BYTE buff[], int num);
	void RecvDatafromIF(void);
	BYTE RecvDatatoBuff(BYTE buff[], int num);
	BYTE isBMRun(void) { if(b_BMRunOK) return ON; else return OFF;};
	void resetBMRunFlag(void) { b_BMRunOK = OFF; };
	
    serial::Serial mc_serial;

    // ROS 2 publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sonar_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hw_version_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fw_version_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sw_version_pub;
	//ros::Publisher line_detect_pub;
	
	
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
	
	BYTE checksum;

	std::string port_name;

	std::thread recv_thread;

	sensor_msgs::msg::Imu imu_msgs;
	std_msgs::msg::MultiArrayDimension sonar_msg_dim;
	std_msgs::msg::Float32MultiArray sonar_msgs;
	std_msgs::msg::String hw_version_msgs;
	std_msgs::msg::String fw_version_msgs;
	std_msgs::msg::String sw_version_msgs;
	//std_msgs::UInt8MultiArray line_detect_msgs;


};

#endif