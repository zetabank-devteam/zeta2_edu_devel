#include "zeta_mc.hpp"

// #define DEBUG

Zeta_MC::Zeta_MC(const std::string& node_name) :rclcpp::Node(node_name)
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

Zeta_MC::~Zeta_MC()
{

	b_SerialThreadRun = false;

    if (recv_thread.joinable()) {
        recv_thread.join();
    }

	if(b_serialOpenOK)
		mc_serial.close();
}

void Zeta_MC::Init(void)
{
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB1");
    this->declare_parameter<int>("baudrate", 115200);

    port_name = this->get_parameter("port_name").as_string();
    nBaudrate = this->get_parameter("baudrate").as_int();

    RCLCPP_INFO(this->get_logger(), "port name : %s.", port_name.c_str());
    RCLCPP_INFO(this->get_logger(), "baudrate : %d.", nBaudrate);

    mc_serialout_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("motor_driver_serial_output", 1);
	fw_version_pub = this->create_publisher<std_msgs::msg::String>("driver_fw_version", 1);
	sw_version_pub = this->create_publisher<std_msgs::msg::String>("driver_sw_version", 1);
    
	mc_serialin_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "motor_driver_serial_input",
        50,
        std::bind(&Zeta_MC::MCSerialInputCallback, this, std::placeholders::_1)
    );

	// mc_serialout_msgs.data = (uint8_t*)malloc(30);
	// mc_serialout_msgs.data = (uint8_t*)malloc(sizeof(uint8_t)*MAX_PACKET_SIZE);
	fw_version_msgs.data = FW_VERSION;
	sw_version_msgs.data = SW_VERSION;

	mc_serialout_msg_dim.label = "robot_pos_vel";       
    mc_serialout_msg_dim.size = 1;                     
    mc_serialout_msgs.layout.dim.clear();
    mc_serialout_msgs.layout.dim.push_back(mc_serialout_msg_dim);  

	// mc_serialout_msgs.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
	// mc_serialout_msgs.layout.dim[0].label = "vel";
	// mc_serialout_msgs.layout.dim[0].size = MAX_PACKET_SIZE;
	// mc_serialout_msgs.layout.dim[0].stride = 1;
	// mc_serialout_msgs.layout.data_offset = 0;
	// mc_serialout_msgs.data = (uint8_t *)malloc(sizeof(uint8_t)*MAX_PACKET_SIZE);
	// mc_serialout_msgs.data_length = MAX_PACKET_SIZE;
   
}

int Zeta_MC::InitSerial(void)
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
		
        RCLCPP_INFO(this->get_logger(), "Zeta Motor Control Serial Port initialized");

		recvThread_Run();

        RCLCPP_INFO(this->get_logger(), "Run serial receive thread...");

	}
    else
        return -1;
}

void Zeta_MC::recvThread_Run(void)
{
	b_SerialThreadRun = true;

    recv_thread = std::thread(&Zeta_MC::RecvDatafromMC, this);

}

void Zeta_MC::RecvDatafromMC(void)
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


void Zeta_MC::ClearRecvBuff(void)
{
	BYTE rbuff[100];
	BYTE rnum = 0;

	if(mc_serial.available() > 0)
	{
		mc_serial.read(rbuff, rnum);
        RCLCPP_INFO(this->get_logger(), "Receive data of initial serial buffer...");
	}

	SetZeroMem(recvBuffer, MAX_RECVBUFF_SIZE);

}


void Zeta_MC::SetZeroMem(BYTE buff[], int num)
{
	int i;
	for(i = 0; i < num; i++)
		buff[i] = 0;
}

BYTE Zeta_MC::RecvDatatoBuff(BYTE buff[], int num)
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
			//RCLCPP_INFO(this->get_logger(), "(%d) : %d ", i, buff[i]);

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

				// RCLCPP_INFO(this->get_logger(), "start : rsize(%d) ", rsize);
				// } 
				// else 
				// {
				// 	recvBuffer[rsize++] = buff[i];

				// RCLCPP_INFO(this->get_logger(), "[2](%d) : %d ", i, buff[i]);
				// }
			} else if(b_StartFlag == true){				

				if(rsize == 3)
					cmd = buff[i];

				if(rsize == 4)
					pid_monitoring = buff[i];

				if(rsize == 5)
					monitoring_unit = buff[i];

				recvBuffer[rsize++] = buff[i];				
#ifdef DEBUG		
				RCLCPP_INFO(this->get_logger(), "[2](%d) : %d recvBuffer(%d):%d", i, buff[i], rsize-1, recvBuffer[rsize-1]);
#endif
				if(buff[i] == 0x3B)
				{
#ifdef DEBUG							
					RCLCPP_INFO(this->get_logger(), "receive end : rsize(%d) cmd:%d recvBuffer[3]:%d", rsize, cmd, recvBuffer[3]);
#endif
					//RCLCPP_INFO(this->get_logger(), "receive end : rsize(%d) cmd:%d pid_monitoring:%d monitoring_unit:%d", rsize, cmd, pid_monitoring, monitoring_unit);

					if(cmd == GET_POSVEL)
					{
						if((pid_monitoring == 0) && (monitoring_unit == 3))
						{
#ifdef DEBUG								
							RCLCPP_INFO(this->get_logger(), "inside 0x0c");
	#endif
							//memcpy((uint8_t *)(mc_serialout_msgs.data), &(uint8_t *)(recvBuffer[j+4]), sizeof(uint8_t)*12);
							mc_serialout_msgs.data.clear(); 

							for(int j=0; j< 12; j++)
							{
								//mc_serialout_msgs.data[j]	= (uint8_t)recvBuffer[j+4];
								mc_serialout_msgs.data.push_back(recvBuffer[j+4]);
#ifdef DEBUG								
								RCLCPP_INFO(this->get_logger(), "mc_serialout_msgs.data[%d]:%d recvBuffer[%d]:%d", j, mc_serialout_msgs.data[j], j+4, recvBuffer[j+4]);
#endif								
							}

							mc_serialout_pub->publish(mc_serialout_msgs);
#ifdef DEBUG								
							RCLCPP_INFO(this->get_logger(), "send zeta motor vel & pos data...");
#endif						
						}
					} else if(cmd == GET_FW_VER)
					{
						//memcpy(mc_serialout_msgs.data, recvBuffer[j+4], sizeof(uint8_t)*5);

						if((pid_monitoring == 0) && (monitoring_unit == 3))
						{
							for(int j=0; j<6; j++)
								fw_version_msgs.data[j] = recvBuffer[j + 6];

							fw_version_pub->publish(fw_version_msgs);
							sw_version_pub->publish(sw_version_msgs);
#ifdef DEBUG								
							RCLCPP_INFO(this->get_logger(), "send zeta motor fw version data...");
#endif						
						}
					}	

					rsize = 0;
					b_StartFlag = false;

					SetZeroMem(recvBuffer, MAX_RECVBUFF_SIZE);
#ifdef DEBUG
					RCLCPP_INFO(this->get_logger(), "0x3b : reset recvBuffer...");
#endif					
				}
			}
		}

	} else{
		rsize = 0;
		b_StartFlag = false;

		SetZeroMem(recvBuffer, MAX_RECVBUFF_SIZE);

#ifdef DEBUG
		RCLCPP_INFO(this->get_logger(), "input big size data : reset recvBuffer...");
#endif		
	}
	
	return SUCCESS;	
}

void Zeta_MC::MCSerialInputCallback(const std_msgs::msg::UInt8MultiArray& msg)
{
	int cnt = 0;
	int size = msg.data.size();

	//RCLCPP_INFO(this->get_logger(), "topic data size : %d.", size);

	for(int i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;

	
	sendBuf[0] = 0xAA;
    sendBuf[1] = 0xAA;
    sendBuf[2] = 0xAA;
	sendBuf[3] = VEL_CONTROL;
	sendBuf[4] = size;

	cnt = 5;

	for(int i = 0; i <size; i++)
	{
		sendBuf[i + 5] = msg.data[i];

		cnt++;
	}

	sendBuf[cnt++] = 0x3B;
	sendBuf[cnt] = 0x00;
	
    #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "send data size : %d.", cnt);

    RCLCPP_INFO(this->get_logger(), "Send Data ==>");
    for(int i = 0; i < cnt; i++)
        RCLCPP_INFO(this->get_logger(), "(%d) : %d ", i, sendBuf[i]);
    #endif


	mc_serial.write(sendBuf, cnt);

	memset(sendBuf,0,30);

}
