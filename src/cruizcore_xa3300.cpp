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
	int file_descriptor;
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
		if (-1 == (file_descriptor = open(COMM_PORT, O_RDONLY)))
		{			
			return false;
		}

		lock_ = PTHREAD_MUTEX_INITIALIZER;

		return true;
	}

	void closeSensor()
	{
		close(file_descriptor);
		cout << "Closing Cruizcore XA 3300 Sensor" << endl;
	}

	bool receiveData()
	{
		unsigned char received_data;
		
		// pthread_mutex_lock(&lock_);

		if (read(file_descriptor, &received_data, sizeof(char)) == -1)
		{
			cout << "Data read error !!!\n";
			return false;
		}

			cout << std::hex << (0xFF & received_data) << " ";
		decodingPacket(received_data);

		// pthread_mutex_unlock(&lock_);

		return true;
	}

	void decodingPacket(unsigned char data)
	{
		if (count > 1)
		{
			data_packet[count++] = data;
			if (count == PACKET_SIZE)
			{
				count = 0;

				publishTopic();
			}
		}
		else if (data == 0xFF && count == 0)
		{
			data_packet[count++] = data;
		}
		else if (data == 0xFF && count == 1)
		{
			data_packet[count++] = data;
		}
		else
		{
			count = 0;
		}
	}

	void publishTopic()
	{
		XA3300Packet xa3300Packet;
		memcpy(&xa3300Packet, data_packet, sizeof(XA3300Packet));
		// cout << "SyncByte : " << xa3300Packet.SyncByte << endl;
		// cout << "PacketCounter : " << xa3300Packet.PacketCounter << endl;

		short header;
		char  index;
		short angle_int;
		short rate_int;
		short accel_x_int;
		short accel_y_int;
		short accel_z_int;
		char	Reserved;
		char	check_sum;

		char	i;
		char	temp;
		char	check_sum_cal;
		float rate_float;
		float angle_float;
		float accel_x_float;
		float accel_y_float;
		float accel_z_float;
		// Copy values from data string 
		// memcpy(&index, data_packet + 2, sizeof(char));
		// memcpy(&angle_int, data_packet + 3, sizeof(short));
		// memcpy(&rate_int, data_packet + 5, sizeof(short));
		// memcpy(&accel_x_int, data_packet + 7, sizeof(short));
		// memcpy(&accel_y_int, data_packet + 9, sizeof(short));
		// memcpy(&accel_z_int, data_packet + 11, sizeof(short));
		// memcpy(&Reserved, data_packet + 13, sizeof(char));
		// memcpy(&check_sum, data_packet + 41, sizeof(char));

		check_sum_cal = 0;

		for (i = 0; i < 42; i++)
		{
			memcpy(&temp, data_packet + i + 0, sizeof(char));
			int a = temp;
			// cout << std::hex << (0xFF & temp) << " ";
			check_sum_cal ^= temp;
		}

		// Verify data checksum
		if (check_sum != check_sum_cal)
		{
			// cout << "Checksum error!!\n";
			return;
		} else {
			cout << "Success" << endl;
		}


		// Apply scale factors
		rate_float = rate_int / 100.0;
		angle_float = angle_int / 100.0;
		accel_x_float = accel_x_int / 1000.0;
		accel_y_float = accel_y_int / 1000.0;
		accel_z_float = accel_z_int / 1000.0;

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
		roll = 0.0;
		pitch = 0.0;
		yaw = angle_float * convertor_d2r;

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
		imu_data_msg.linear_acceleration.x = accel_x_float;
		imu_data_raw_msg.linear_acceleration.y =
		imu_data_msg.linear_acceleration.y = accel_y_float;
		imu_data_raw_msg.linear_acceleration.z =
		imu_data_msg.linear_acceleration.z = accel_z_float;

		// imu.gx gy gz.
		// original data used the degree/s unit, convert to radian/s
		imu_data_raw_msg.angular_velocity.x =
		imu_data_msg.angular_velocity.x = 0.0;
		imu_data_raw_msg.angular_velocity.y =
		imu_data_msg.angular_velocity.y = 0.0;
		imu_data_raw_msg.angular_velocity.z =
		imu_data_msg.angular_velocity.z = 0.0;

		// publish the IMU data
		imu_data_raw_pub_.publish(imu_data_raw_msg);
		imu_data_pub_.publish(imu_data_msg);

		// publish tf
		broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
			tf::Vector3(0.0, 0.0, 0.0)),
			ros::Time::now(), parent_frame_id_, frame_id_));

		// publish
		//cout << "rate:" << rate_float << " [deg/sec]\t angle:" << angle_float << " [deg]\n";
		//cout << "accel_x:" << accel_x_float << " [m/sec^2]\t accel_y:" << accel_y_float << " [m/sec^2]\t accel_z:" << accel_z_float << " [m/sec^2]\n";
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

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
	  sensor.receiveData();

	  ros::spinOnce();
  }

  //ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
