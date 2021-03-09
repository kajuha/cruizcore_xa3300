// Microinfinity Cia. Ltd.
// This program demonstrates how to communicate with the CruizCore xa3300 IMU
// This program assumes that the port 'COMM_PORT' has been properly initialized. 
// In linux this can be done is Linux using the following command line:
// 		stty -F /dev/ttyUSB0 115200 raw
// the sensor can be set in "calibration" data mude using:
//	 	echo -e '$MI data cal\r'	> /dev/ttyUSB0
// This program can be compiled using:
// 		g++ -o xa3300 xa3300.cpp

#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>

using namespace std;

#pragma pack(1)
typedef struct _XA3300Packet {
	unsigned short SyncByte;
	unsigned short PacketCounter;
	unsigned char DataInfo;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float AccX;
	float AccY;
	float AccZ;
	float DegRoll;
	float DegPitch;
	float DegYaw;
	unsigned char Checksum;
} XA3300Packet;

//Define constants
const char COMM_PORT[] = "/dev/ttyUSB0";
const int PACKET_SIZE = 42;
const int SAMPLES = 1000;

//Define global variables
int fd;

// Open serial port
bool xa3300_init()
{
	if(-1 == (fd = open(COMM_PORT, O_RDWR)))
	{
		cout << "Error opening port \n";
		cout << "Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 115200 raw\n";
		cout << "You may need to have ROOT access";
		return false;
	}

	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));

	newtio.c_cflag = B115200;
	newtio.c_cflag |= CS8;
	newtio.c_cflag |= CLOCAL;
	newtio.c_cflag |= CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0; 
	// kajuha 20210302
	// newtio.c_cc[VMIN] = 1; 
	newtio.c_cc[VMIN] = 0; 

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	cout << "CruizCoreR1350 communication port is ready\n";
	return true;
}

// Close serial port
void xa3300_close()
{
	close(fd);
	cout << "Closing communication port";
}

// Get a data packet and parse it
bool xa3300_getvalue()
{
	XA3300Packet xA3300Packet;
	short header;
	short check_sum;
	unsigned char data_packet[PACKET_SIZE];
	
	if (PACKET_SIZE != read(fd, data_packet, PACKET_SIZE))
		return false;

	// Verify data packet header 
	memcpy(&header, data_packet, sizeof(short));
	if (header != (short)0xFFFF)
	{
		cout << "Header error !!!\n";
		return false;
	}
	
	// Verify data checksum
	check_sum = data_packet[2];
	for (int i = 3; i<PACKET_SIZE-1; i++)
		check_sum ^= data_packet[i];
	if (check_sum != data_packet[PACKET_SIZE-1])
	{ 
		cout<< "Checksum error!!\n";
		return false;
	}

	memcpy(&xA3300Packet,data_packet,sizeof(xA3300Packet));
	cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << xA3300Packet.RateRoll << " " << setw(8) << xA3300Packet.RatePitch << " " << setw(8) << xA3300Packet.RateYaw << endl;
	cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << xA3300Packet.AccX << " " << setw(8) << xA3300Packet.AccY << " " << setw(8) << xA3300Packet.AccZ << endl;
	cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << xA3300Packet.DegRoll << " " << setw(8) << xA3300Packet.DegPitch << " " << setw(8) << xA3300Packet.DegYaw << endl;
	
	return true;
}

int main()
{
	// Open communication channel
	if(!xa3300_init())
		return 0;

	// Get, parse and display data
	while( 1 )
		xa3300_getvalue();
	
	// Close communication
	xa3300_close();
	return 1;
}
