#include "zeta_if.hpp"

// #define DEBUG

union FLOATVAL {
	char cval[4];
	float fval;
} float_val;

Zeta_IF::Zeta_IF(const std::string& node_name) :rclcpp::Node(node_name)
{
	b_serialOpenOK = false;
	PacketNumber = 0;
	b_recvOK = false;
	b_SerialThreadRun = false;
	b_PacketOK = false;
	b_BMRunOK = false;
	b_StartFlag = false;
	rsize = 0;
	startcnt = 0;
}

Zeta_IF::~Zeta_IF()
{

	b_SerialThreadRun = false;
	
    if (recv_thread.joinable()) {
        recv_thread.join();
    }

	if(b_serialOpenOK)
		mc_serial.close();
}

void Zeta_IF::Init(void)
{
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 115200);

    port_name = this->get_parameter("port_name").as_string();
    nBaudrate = this->get_parameter("baudrate").as_int();

    RCLCPP_INFO(this->get_logger(), "port name : %s.", port_name.c_str());
    RCLCPP_INFO(this->get_logger(), "baudrate : %d.", nBaudrate);

	
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    sonar_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("sonar", 10);
    hw_version_pub = this->create_publisher<std_msgs::msg::String>("core_hw_version", 10);
    fw_version_pub = this->create_publisher<std_msgs::msg::String>("core_fw_version", 10);
    sw_version_pub = this->create_publisher<std_msgs::msg::String>("core_sw_version", 10);

	//line_detect_pub = nh.advertise<std_msgs::UInt8MultiArray>("line_detecting", 1);

	//mc_serialin_sub = nh.subscribe("motor_driver_serial_input", 50, &Zeta_MC::MCSerialInputCallback, this);

	hw_version_msgs.data = HW_VERSION;
	fw_version_msgs.data = FW_VERSION;
	sw_version_msgs.data = SW_VERSION;


	sonar_msg_dim.label = "sonar_sensor_data";       
    sonar_msg_dim.size = 4;                     
    sonar_msgs.layout.dim.clear();
    sonar_msgs.layout.dim.push_back(sonar_msg_dim);  

	// mc_serialout_msgs.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
	// mc_serialout_msgs.layout.dim[0].label = "vel";
	// mc_serialout_msgs.layout.dim[0].size = MAX_PACKET_SIZE;
	// mc_serialout_msgs.layout.dim[0].stride = 1;
	// mc_serialout_msgs.layout.data_offset = 0;
	// mc_serialout_msgs.data = (uint8_t *)malloc(sizeof(uint8_t)*MAX_PACKET_SIZE);
	// mc_serialout_msgs.data_length = MAX_PACKET_SIZE;
   
}


int Zeta_IF::InitSerial(void)
{
	//nBaudrate = 9600;
	serial::Timeout to;

    try
    {
		
		mc_serial.setPort(port_name.c_str());		
		// ld_serial.setPort("/dev/ttyACM0");
		// ld_serial.setPort("/dev/ttyUSB1");
		mc_serial.setBaudrate(nBaudrate);
        // mc_serial.setBaudrate(115200);
		// 556 when baud is 19200, 1.8ms
		//1667 when baud is 57600, 0.6ms
		//2857 when baud is 115200, 0.35ms
		if(nBaudrate == 19200)
			to = serial::Timeout::simpleTimeout(556); 
		else if(nBaudrate == 57600)
			to = serial::Timeout::simpleTimeout(1667); 		
		else
        	to = serial::Timeout::simpleTimeout(2857); 
		// serial::Timeout to = serial::Timeout::simpleTimeout(556); 
		// serial::Timeout to = serial::Timeout::simpleTimeout(1667); 
        mc_serial.setTimeout(to);
        mc_serial.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port");
        return -1;
    }
    if(mc_serial.isOpen())
	{
		b_serialOpenOK = true;
		
        RCLCPP_INFO(this->get_logger(), "Zeta Motor Interface Port initialized");

		recvThread_Run();

        RCLCPP_INFO(this->get_logger(), "Run serial receive thread...");

	}
    else
        return -1;
}

void Zeta_IF::recvThread_Run(void)
{
	b_SerialThreadRun = true;

    recv_thread = std::thread(&Zeta_IF::RecvDatafromIF, this);
}

void Zeta_IF::RecvDatafromIF(void)
{
	BYTE rbuff[MAX_RECVBUFF_SIZE];
	BYTE rnum = 0;
	BYTE ret;

	rclcpp::Rate rec_dealy(1000);

	step = 0;

	startcnt = 0;
	b_StartFlag = false;
	rsize = 0;
	
	while(b_SerialThreadRun)
	{
		if(b_serialOpenOK) {
			rnum = mc_serial.available();
			
			if(rnum > 0)
			{
				SetZeroMem(rbuff, MAX_RECVBUFF_SIZE);
				
				mc_serial.read(rbuff, rnum);

				ret = RecvDatatoBuff(rbuff, rnum);
				
			}
		}		

		rec_dealy.sleep();
	}
}


void Zeta_IF::ClearRecvBuff(void)
{
	BYTE rbuff[1000];
	BYTE rnum = 0;

	if(mc_serial.available() > 0)
	{
		mc_serial.read(rbuff, rnum);
        RCLCPP_INFO(this->get_logger(), "Receive data of initial serial buffer...");
	}

	SetZeroMem(recvBuffer, MAX_RECVBUFF_SIZE);

}


void Zeta_IF::SetZeroMem(BYTE buff[], int num)
{
	int i;
	for(i = 0; i < num; i++)
		buff[i] = 0;
}

BYTE Zeta_IF::RecvDatatoBuff(BYTE buff[], int num)
{
	std::stringstream ss;
	int i;
	//int cmd_pos = 0;

	if((num > 0) && (num < 260))
	{
#ifdef DEBUG		
		RCLCPP_INFO(this->get_logger(), "[0]receive num: %d", num);

#endif

		//rsize = 0;
		//b_StartFlag = false;
		//startcnt = 0;

		for(i = 0; i < num; i++)		
		{
			//ROS_INFO("(%d) : %d ", i, buff[i]);

			if((buff[i] == 0xAA) && (b_StartFlag == false))
			{
				//startcnt++;

				// if((startcnt == 1) && (b_StartFlag == false))
				// {
				// 	rsize = 0;
				// }
#ifdef DEBUG		
				RCLCPP_INFO(this->get_logger(), "[1](%d) : %d rsize(%d) startcnt(%d)", i, buff[i], rsize, startcnt);
#endif
				recvBuffer[rsize++] = buff[i];

				if(buff[i] == 0xAA)
					startcnt++;

				if(startcnt >= 3)
				{
					startcnt = 0;
					b_StartFlag = true;

					//rsize = 3;

					//cmd_pos = rsize;
				}

				// if(startcnt >= 3)
				// {
				// 	startcnt = 0;
				// 	b_StartFlag = true;

				// 	//recvBuffer[rsize++] = buff[i];

				// 	//if(rsize > 3)
				// 	//{
				// 		//rsize = 0;
				// 		recvBuffer[0] = 0xAA;
				// 		recvBuffer[1] = 0xAA;
				// 		recvBuffer[2] = 0xAA;
				// 		rsize = 3;
				// 	//}

				// 	ROS_INFO("start : rsize(%d) ", rsize);
				// } 
				// else 
				// {
				// 	recvBuffer[rsize++] = buff[i];

				// 	ROS_INFO("[2](%d) : %d ", i, buff[i]);
				// }
			} else if(b_StartFlag == true){				

				if(rsize == 3)
					cmd = buff[i];
				
				// ROS_INFO("==> rsize(%d) ", rsize);

				recvBuffer[rsize++] = buff[i];				
#ifdef DEBUG		
				RCLCPP_INFO(this->get_logger(), "[2](%d) : %d recvBuffer(%d):%d", i, buff[i], rsize-1, recvBuffer[rsize-1]);
#endif
				if((buff[i-1] == 0xAA) && (buff[i] == 0xEE))
				// if((buff[i] == 0x3B) && (rsize >= 45))
				{
#ifdef DEBUG							
					RCLCPP_INFO(this->get_logger(), "receive end : rsize(%d) cmd:%d recvBuffer[3]:%d", rsize, cmd, recvBuffer[3]);
#endif
					//ROS_INFO("receive end : rsize(%d) cmd:%d pid_monitoring:%d monitoring_unit:%d", rsize, cmd, pid_monitoring, monitoring_unit);

					if(cmd == SEND_IMU_DATA)
					{												
						//memcpy((uint8_t *)(mc_serialout_msgs.data), &(uint8_t *)(recvBuffer[j+4]), sizeof(uint8_t)*12);
						checksum = 0;
						
					    imu_msgs.header.stamp = this->get_clock()->now();

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[4+j];

						imu_msgs.orientation.w = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[8+j];

						imu_msgs.orientation.x = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[12+j];

						imu_msgs.orientation.y = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[16+j];

						imu_msgs.orientation.z = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[20+j];

						imu_msgs.angular_velocity.x = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[24+j];

						imu_msgs.angular_velocity.y = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[28+j];

						imu_msgs.angular_velocity.z = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[32+j];

						imu_msgs.linear_acceleration.x = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[36+j];

						imu_msgs.linear_acceleration.y = float_val.fval;

						for(int j=0; j< 4; j++)														
							float_val.cval[j] = recvBuffer[40+j];

						imu_msgs.linear_acceleration.z = float_val.fval;
						
#ifdef DEBUG								
						RCLCPP_INFO(this->get_logger(), "[%d] ==> orientation.w(%f) orientation.x(%f) orientation.y(%f) orientation.z(%f)", j, imu_msgs.orientation.w, imu_msgs.orientation.x, imu_msgs.orientation.y, imu_msgs.orientation.z);

						RCLCPP_INFO(this->get_logger(), "[%d] ==> angular_velocity.x(%f) angular_velocity.y(%f) angular_velocity.z(%f)", j, imu_msgs.angular_velocity.x, imu_msgs.angular_velocity.y, imu_msgs.angular_velocity.z);

						RCLCPP_INFO(this->get_logger(), "[%d] ==> linear_acceleration.x(%f) linear_acceleration.y(%f) linear_acceleration.z(%f)", j, imu_msgs.linear_acceleration.x, imu_msgs.linear_acceleration.y, imu_msgs.linear_acceleration.z);

						RCLCPP_INFO(this->get_logger(), "Receive end: size(%d)", rsize);
						// ROS_INFO("receive end : rsize(%d) cmd:%d recvBuffer[3]:%d", rsize, cmd, recvBuffer[3]);
#endif						

						for(int j=3; j<44; j++)
							checksum += recvBuffer[j];
						
						if(checksum == recvBuffer[44])
						{
							imu_pub->publish(imu_msgs);
#ifdef DEBUG								
							RCLCPP_INFO(this->get_logger(), "publish zeta imu data...");
#endif					
						}	
						else
						{
#ifdef DEBUG				
							RCLCPP_INFO(this->get_logger(), "check sum error of imu data...");
#endif					
						}

#ifdef DEBUG								
						RCLCPP_INFO(this->get_logger(), "Input count: %d", i);
#endif					
						
					} 
					else if(cmd == SEND_SONAR_DATA)
					{												
						//memcpy((uint8_t *)(mc_serialout_msgs.data), &(uint8_t *)(recvBuffer[j+4]), sizeof(uint8_t)*12);
						sonar_msgs.data.clear(); 

						for(int j=0; j< 4; j++)	
						{
							for(int k=0; k< 4; k++)	
								float_val.cval[k] = recvBuffer[4*j+k];

							sonar_msgs.data[j] = float_val.fval;
							// sonar_msgs.data = push_back(float_val.fval);
#ifdef DEBUG								
							RCLCPP_INFO(this->get_logger(), "sonar_msgs.data[%d]: %f", j, sonar_msgs.data[j]);
#endif								
						}

						sonar_pub->publish(sonar_msgs);
#ifdef DEBUG								
						RCLCPP_INFO(this->get_logger(), "Send Zeta sonar sensor data...");
#endif						
						
					} 
					else if(cmd == SEND_HWFW_VER)
					{
						//memcpy(mc_serialout_msgs.data, recvBuffer[j+4], sizeof(uint8_t)*5);
						
						for(int j=0; j<6; j++)
							hw_version_msgs.data[j] = recvBuffer[j + 5];

						for(int j=0; j<6; j++)
							fw_version_msgs.data[j] = recvBuffer[j + 12];

						hw_version_pub->publish(hw_version_msgs);
						fw_version_pub->publish(fw_version_msgs);
						sw_version_pub->publish(sw_version_msgs);
#ifdef DEBUG								
						RCLCPP_INFO(this->get_logger(), "Send Zeta motor hw/fw version data...");
#endif						
						
					}	
					/*else if(cmd == SEND_LINEDETECT)
					{												
						//memcpy((uint8_t *)(line_detect_msgs.data), &(uint8_t *)(recvBuffer[j+4]), sizeof(uint8_t)*12);
						line_detect_msgs.data.clear(); 

						for(int j=0; j< 4; j++)	
						{
							line_detect_msgs.data = push_back(recvBuffer[4+j]);
#ifdef DEBUG								
							ROS_INFO("line_detect_msgs.data[%d]:%d  recvBuffer[%d]:%d", j, line_detect_msgs.data[j], j+4, recvBuffer[j+4]);
#endif								
						}

						line_detect_pub.publish(line_detect_msgs);
#ifdef DEBUG								
						ROS_INFO("send zeta line detection data...");
#endif												
					} */

					rsize = 0;
					b_StartFlag = false;

					SetZeroMem(recvBuffer, 260);					

#ifdef DEBUG
					RCLCPP_INFO(this->get_logger(), "0x3B: Reset recvBuffer...");
#endif					
				}				
			}
		}

	} else{
		rsize = 0;
		b_StartFlag = false;

		SetZeroMem(recvBuffer, 260);

#ifdef DEBUG
		RCLCPP_INFO(this->get_logger(), "Input big size data: Reset recvBuffer...");
#endif		
	}
	
	return SUCCESS;	
}


