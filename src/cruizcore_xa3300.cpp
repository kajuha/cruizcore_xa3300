#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <iostream>

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

class CruizcoreDriverForROS
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	ros::Publisher imu_data_raw_pub_;
	ros::Publisher imu_data_pub_;

	tf::TransformBroadcaster broadcaster_;

	pthread_mutex_t lock_;

	std::string parent_frame_id_;
	std::string frame_id_;
	double linear_acceleration_stddev_;		// need check.
	double angular_velocity_stddev_;		// need check.

	//Define constants
	const char* COMM_PORT = "/dev/ttyUSB0";
	const static int PACKET_SIZE = 42;
	const int SAMPLES = 1000;

	//Define global variables
	int fd;
	unsigned char data_packet[PACKET_SIZE];
	int count = 0;


public:
	CruizcoreDriverForROS(std::string port = "/dev/ttyUSB0", int baud_rate = 115200)
		: nh_priv_("~")
	{
		// dependent on user device
		nh_priv_.setParam("port", port);
		nh_priv_.setParam("baud_rate", baud_rate);
		
		// default frame id
		nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
		
		// for testing the tf
		nh_priv_.param("parent_frame_id_", parent_frame_id_, std::string("base_link"));
		
		// publisher for streaming
		imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
		imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
	}

	~CruizcoreDriverForROS()
	{}

	bool initialize()
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
		newtio.c_cc[VMIN] = 1; 

		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);

		cout << "CruizCoreR1350 communication port is ready\n";

		lock_ = PTHREAD_MUTEX_INITIALIZER;

		return true;
	}

	void closeSensor()
	{
		close(fd);
		cout << "Closing Cruizcore XA 3300 Sensor" << endl;
	}

	bool receiveData()
	{
		short header;
		short check_sum;
		
		// pthread_mutex_lock(&lock_);
	
		if (PACKET_SIZE != read(fd, data_packet, PACKET_SIZE)) {
			cout << "Receive Fail !!!\n";
			return false;
		}

		// Verify data packet header 
		memcpy(&header, data_packet, sizeof(short));
		if (header != (short)0xFFFF) {
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

		publishTopic();

		// pthread_mutex_unlock(&lock_);

		return true;
	}

	void publishTopic()
	{
		XA3300Packet xa3300Packet;
		memcpy(&xa3300Packet, data_packet, sizeof(XA3300Packet));

		// cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << xa3300Packet.RateRoll << " " << setw(8) << xa3300Packet.RatePitch << " " << setw(8) << xa3300Packet.RateYaw << endl;
		// cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << xa3300Packet.AccX << " " << setw(8) << xa3300Packet.AccY << " " << setw(8) << xa3300Packet.AccZ << endl;
		// cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << xa3300Packet.DegRoll << " " << setw(8) << xa3300Packet.DegPitch << " " << setw(8) << xa3300Packet.DegYaw << endl;
	
		// Publish ROS msgs.
		sensor_msgs::Imu imu_data_raw_msg;
		sensor_msgs::Imu imu_data_msg;

		// Set covariance value of each measurements.
		imu_data_raw_msg.linear_acceleration_covariance[0] =
		imu_data_raw_msg.linear_acceleration_covariance[4] =
		imu_data_raw_msg.linear_acceleration_covariance[8] =
		imu_data_msg.linear_acceleration_covariance[0] =
		imu_data_msg.linear_acceleration_covariance[4] =
		imu_data_msg.linear_acceleration_covariance[8] = -1;

		imu_data_raw_msg.angular_velocity_covariance[0] =
		imu_data_raw_msg.angular_velocity_covariance[4] =
		imu_data_raw_msg.angular_velocity_covariance[8] =
		imu_data_msg.angular_velocity_covariance[0] =
		imu_data_msg.angular_velocity_covariance[4] =
		imu_data_msg.angular_velocity_covariance[8] = -1;

		imu_data_msg.orientation_covariance[0] =
		imu_data_msg.orientation_covariance[4] =
		imu_data_msg.orientation_covariance[8] = -1;

		static double convertor_d2r = M_PI / 180.0; // for angular_velocity (degree to radian)
		static double convertor_r2d = 180.0 / M_PI; // for easy understanding (radian to degree)

		double roll, pitch, yaw;
		roll = xa3300Packet.DegRoll * convertor_d2r;
		pitch = xa3300Packet.DegPitch * convertor_d2r;
		yaw = xa3300Packet.DegYaw * convertor_d2r;

		// Get Quaternion fro RPY.
		tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

		ros::Time now = ros::Time::now();

		imu_data_raw_msg.header.stamp =
		imu_data_msg.header.stamp = now;
		
		imu_data_raw_msg.header.frame_id =
		imu_data_msg.header.frame_id = frame_id_;

		// orientation
		imu_data_msg.orientation.x = orientation[0];
		imu_data_msg.orientation.y = orientation[1];
		imu_data_msg.orientation.z = orientation[2];
		imu_data_msg.orientation.w = orientation[3];

		// original data used the g unit, convert to m/s^2
		imu_data_raw_msg.linear_acceleration.x =
		imu_data_msg.linear_acceleration.x = 0;
		imu_data_raw_msg.linear_acceleration.y =
		imu_data_msg.linear_acceleration.y = 0;
		imu_data_raw_msg.linear_acceleration.z =
		imu_data_msg.linear_acceleration.z = 0;
		imu_data_raw_msg.linear_acceleration.x =
		imu_data_msg.linear_acceleration.x = xa3300Packet.AccX;
		imu_data_raw_msg.linear_acceleration.y =
		imu_data_msg.linear_acceleration.y = xa3300Packet.AccY;
		imu_data_raw_msg.linear_acceleration.z =
		imu_data_msg.linear_acceleration.z = xa3300Packet.AccZ;

		// imu.gx gy gz.
		// original data used the degree/s unit, convert to radian/s
		imu_data_raw_msg.angular_velocity.x =
		imu_data_msg.angular_velocity.x = 0;
		imu_data_raw_msg.angular_velocity.y =
		imu_data_msg.angular_velocity.y = 0;
		imu_data_raw_msg.angular_velocity.z =
		imu_data_msg.angular_velocity.z = 0;
		imu_data_raw_msg.angular_velocity.x =
		imu_data_msg.angular_velocity.x = xa3300Packet.RateRoll * convertor_d2r;
		imu_data_raw_msg.angular_velocity.y =
		imu_data_msg.angular_velocity.y = xa3300Packet.RatePitch * convertor_d2r;
		imu_data_raw_msg.angular_velocity.z =
		imu_data_msg.angular_velocity.z = xa3300Packet.RateYaw * convertor_d2r;

		// publish the IMU data
		imu_data_raw_pub_.publish(imu_data_raw_msg);
		imu_data_pub_.publish(imu_data_msg);

		// publish tf
		broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
			tf::Vector3(0.0, 0.0, 0.0)),
			ros::Time::now(), parent_frame_id_, frame_id_));
	}

};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cruizcore_xa3300");

  std::string port = std::string("/dev/ttyUSB0");
  int baud_rate    = 115200;

  ros::param::get("~port", port);
  ros::param::get("~baud_rate", baud_rate);

  CruizcoreDriverForROS sensor(port, baud_rate);

  if(sensor.initialize() == false)
  {
    ROS_ERROR("Initialize() returns false, please check your devices.\n");
	ROS_ERROR("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 115200 raw\n");
	ROS_ERROR("You may need to have ROOT access\n");
    return 0;
  }
  else
  {
    ROS_INFO("CruizCore XA 3300 Initialization OK!\n");
  }

  // ros::Rate loop_rate(10);

  while (ros::ok())
  {
	  sensor.receiveData();

	  ros::spinOnce();
  }

  //ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
