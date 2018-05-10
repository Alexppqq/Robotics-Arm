/*04/23/2018*/
/*
Centriod:
DX1  2048
DX2  2048
DX3  2048
DX4  2048
DX5  512
DX6  512
*/
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <windows.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
using namespace std;
#define ADDR_DXL_CW_ANGLE_LIMIT		6
#define ADDR_DXL_CCW_ANGLE_LIMIT	8
// Control table address
//AX-12
#define ADDR_AX_TORQUE_ENABLE           0X18                  // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           0X1E
#define ADDR_AX_PRESENT_POSITION        0X24
#define ADDR_AX_GOAL_SPEED              0X20
#define ADDR_AX_PRESENT_SPEED           0X26
#define ADDR_AX_GOAL_TORQUE             0X22
//MX-64 
// Control table address same with AX-12
#define ADDR_MX_TORQUE_ENABLE           0X18                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           0X1E
#define ADDR_MX_PRESENT_POSITION        0X24
#define ADDR_MX_GOAL_SPEED              0X20
#define ADDR_MX_PRESENT_SPEED           0X26
#define ADDR_MX_GOAL_TORQUE             0X22
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


int read_status(int dxl_id, int address);
int set_pos( int goal_pos, int dxl_id);
int set_vel(int goal_vel, int dxl_id);
int set_torque(int goal_torque, int dxl_id);
int disable_torque(int dxl_id);
int set_mode(int mode, int dxl_id);  //0 is wheel mode and 1 is joint mode
int set_pos_array(int* goal_pos_array, int num);
int set_vel_array(int* goal_vel_array, int num);
int set_torque_array(int* goal_torque_array, int num);
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}


int set_pos( int goal_pos, int dxl_id)
{
	int addr_goal_pos=ADDR_AX_GOAL_POSITION;
	int addr_pre_pos=ADDR_AX_PRESENT_POSITION;
	int move_threshold= DXL_MOVING_STATUS_THRESHOLD;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_present_position = 0;              // Present position
	uint16_t dxl_present_speed = 0;              // Present position
												 // Open port
	
		// Write goal position
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr_goal_pos, goal_pos, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		}

		do
		{
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, addr_pre_pos, &dxl_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}

			//printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl_id, goal_pos, dxl_present_position);

		} while ((abs(goal_pos - dxl_present_position) > move_threshold));

	return 0;
}

int read_status(int dxl_id, int address)
{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_present_status = 0;              // Present status

		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, address, &dxl_present_status, &dxl_error);
		printf("#################PresStatus is :%03d\n", dxl_present_status);
	return 0;
}
int set_vel(int goal_vel, int dxl_id)
{
	int addr_goal_speed = ADDR_AX_GOAL_SPEED;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error

	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr_goal_speed, goal_vel, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	return 0;
}
int set_torque(int goal_torque, int dxl_id)
{
	int addr_torque_enable = ADDR_AX_TORQUE_ENABLE;
	int addr_goal_torque = ADDR_AX_GOAL_TORQUE;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error

													// Enable Dynamixel Torque
	
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_torque_enable, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel has been successfully connected \n");
	}

	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr_goal_torque, goal_torque, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	return 0;
}
int disable_torque(int dxl_id)
{
	int addr_torque_enable = ADDR_AX_TORQUE_ENABLE;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_torque_enable, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	return 0;
}
int set_pos_array(int* goal_pos_array, int num)
{
	if (num != 5)
	{
		printf("You need input 5 elemetns, from joint 1 to joint 5\n");
	}
	int dxl_id = 0;
	int addr_goal_pos = ADDR_AX_GOAL_POSITION;
	int addr_pre_pos = ADDR_AX_PRESENT_POSITION;
	int move_threshold = DXL_MOVING_STATUS_THRESHOLD;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_present_position = 0;              // Present position
	uint16_t dxl_present_speed = 0;              // Present position
												 // Open port

												 // Write goal position
	int goal_pos = 0;
	for (int i=0;i<num;i++)
	{
		goal_pos = goal_pos_array[i];
		dxl_id = i + 1;
		set_pos(goal_pos, dxl_id);

	}

	return 0;
}
int set_vel_array(int* goal_vel_array, int num)
{
	if (num != 5)
	{
		printf("You need input 5 elemetns, from joint 1 to joint 5\n");
	}
	int dxl_id = 0;
	int addr_goal_speed = ADDR_AX_GOAL_SPEED;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error

	int goal_vel = 0;
	for (int i = 0; i < num; i++)
	{
		goal_vel = goal_vel_array[i];
		dxl_id = i + 1;
		set_vel(goal_vel, dxl_id);
	}
	return 0;
}
int set_torque_array(int* goal_torque_array, int num)
{
	if (num != 5)
	{
		printf("You need input 5 elemetns, from joint 1 to joint 5\n");
	}
	int dxl_id = 0;
	int addr_torque_enable = ADDR_AX_TORQUE_ENABLE;
	int addr_goal_torque = ADDR_AX_GOAL_TORQUE;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error


	int goal_torque = 0;
	for (int i = 0; i < num; i++)
	{
		goal_torque = goal_torque_array[i];
		dxl_id = i + 1;
		set_torque(goal_torque, dxl_id);
	}


	return 0;
}


int set_mode(int mode, int dxl_id)
{
	int addr_goal_speed = ADDR_AX_GOAL_SPEED;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;   // Dynamixel error
	int id = dxl_id;
	int cw = 0;
	int ccw = 0;
	if (id == 1 && mode==1)
	{
		cw = 0;
		ccw = 4095;
	}
	if (id == 2 && mode == 1)
	{
		cw = 0;
		ccw = 4095;
	}
	if (id == 3 && mode == 1)
	{
		cw = 0;
		ccw = 4095;
	}
	if (id == 4 && mode == 1)
	{
		cw = 0;
		ccw = 4095;
	}
	if (id == 5 && mode == 1)
	{
		cw = 0;
		ccw = 1023;
	}
	if (id == 6 && mode == 1)
	{
		cw = 0;
		ccw = 1023;
	}
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_DXL_CW_ANGLE_LIMIT, cw, &dxl_error);
	uint16_t dxl_present_velocity = 0;              // Present position
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel CW angle set successfully \n");
	}

	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_DXL_CCW_ANGLE_LIMIT, ccw, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel CCW angle set successfully \n");
	}



	return 0;
}
int main()
{
	/*Initialization*/
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

//////////wheel mode/////////////////////////
	set_mode(0, 5);  //change id-5 to wheel mode
	set_vel(89, 5);  //set vel=89 (id 5) and keep moving
	Sleep(3000);  //wait 3 seconds
	set_vel(200, 5);  //change velocity to 200
	set_torque(500, 5);// set torque=500 and use default velocity , keep running
//////////////joint mode : position and array position/////////////////////////////
	//set_mode(1, 1);  //set id 1 motor to joint mode
	//set_mode(1, 2);  //set id 2 motor to joint mode
	//set_mode(1, 3);
	//set_mode(1, 4);
	//set_mode(1, 5);
////////////for single joint////////////////////
	//set_vel(50, 4);   //set id 4 motor' vel to 50
	//set_pos(1900, 4); //move to 1900
	//read_status(4, ADDR_MX_GOAL_POSITION);
/////////////for position/velocity/torque array////////////////////////////////////
	//int pos[5] = { 1900,2000,1900,2000,700 };
	//int pos[5] = { 2048,2048,2048,2048,512 }; //centriod position
	//int vel[5] = { 50,50,50,50,50 };
	//int torque[5] = { 300,300,400,500,400 };
	//set_torque_array(torque, 5);
	//set_vel_array(vel, 5);
	//set_pos_array(pos, 5);


// Close port
	portHandler->closePort();

	system("pause");
	return 0;
}
