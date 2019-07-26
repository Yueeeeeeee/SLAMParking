
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <string.h>
#include "ip_connection.h"
#include "brick_imu_v2.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define HOST "192.168.1.143"
#define PORT 4223

std::string frameId_ = "imu_link";
ros::Publisher imuPub_;
ros::Publisher mfPub_;
ros::Publisher odomPub_;
ros::Time current_time, last_time;

const double deg2rad = M_PI/180.0;

double cov_orientation = 0.05;
double cov_velocity = 0.025;
double cov_acceleration = 0.1;
const int INPUT_BUFFER_SIZE  = 1;

bool removeGravitationalAcceleration_ = false;

// Global Odometry Position 
double x_pos = 0.0;
double y_pos = 0.0;
double th = 0.0;


// Callback function for all data callback
void cb_all_data(int16_t acceleration[3], int16_t magnetic_field[3],
                 int16_t angular_velocity[3], int16_t euler_angle[3],
                 int16_t quaternion[4], int16_t linear_acceleration[3],
                 int16_t gravity_vector[3], int8_t temperature,
                 uint8_t calibration_status, void *user_data) {
	(void)user_data; // avoid unused parameter warning

	ROS_INFO("\nAcceleration        x: %.02f y: %.02f z: %.02f m/s^2\n"
	       "Magnetic Field      x: %.02f y: %.02f z: %.02f micro T\n"
	       "Angular Velocity    x: %.02f y: %.02f z: %.02f deg/s\n"
	       "Euler Angle         x: %.02f y: %.02f z: %.02f deg\n"
	       "Quaternion          x: %.02f y: %.02f z: %.02f w: %.02f\n"
	       "Linear Acceleration x: %.02f y: %.02f z: %.02f m/s^2\n"
	       "Gravity Vector      x: %.02f y: %.02f z: %.02f m/s^2\n"
	       "Temperature         %d deg C\n"
	       "Calibration Status  %d\n\n",
	       acceleration[0]/100.0,        acceleration[1]/100.0,        acceleration[2]/100.0,
	       magnetic_field[0]/16.0,       magnetic_field[1]/16.0,       magnetic_field[2]/16.0,
	       angular_velocity[0]/16.0,     angular_velocity[1]/16.0,     angular_velocity[2]/16.0,
	       euler_angle[0]/16.0,          euler_angle[1]/16.0,          euler_angle[2]/16.0,
	       quaternion[1]/16383.0,        quaternion[2]/16383.0,        quaternion[3]/16383.0,        quaternion[0]/16383.0,
	       linear_acceleration[0]/100.0, linear_acceleration[1]/100.0, linear_acceleration[2]/100.0,
	       gravity_vector[0]/100.0,      gravity_vector[1]/100.0,      gravity_vector[2]/100.0,
	       temperature,
	       calibration_status);

	current_time = ros::Time::now();

	if(imuPub_.getNumSubscribers())
	{
		sensor_msgs::Imu imuMsg;
		imuMsg.header.frame_id = frameId_;
		imuMsg.header.stamp = ros::Time::now();
		boost::array<const double, 9> zeros =
						{ 0, 0, 0,
						  0, 0, 0,
						  0, 0, 0};
		/* Original
		imuMsg.orientation.w = quaternion[0]/16383.0;
		imuMsg.orientation.x = quaternion[1]/16383.0;
		imuMsg.orientation.y = quaternion[2]/16383.0;
		imuMsg.orientation.z = quaternion[3]/16383.0;
		 */
		
		imuMsg.orientation.w = -quaternion[2]/16383.0;
		imuMsg.orientation.x = -quaternion[3]/16383.0;
		imuMsg.orientation.y = quaternion[0]/16383.0;
		imuMsg.orientation.z = quaternion[1]/16383.0;

		imuMsg.orientation_covariance = zeros;
		imuMsg.orientation_covariance[0] = cov_orientation;
		imuMsg.orientation_covariance[4] = cov_orientation;
		imuMsg.orientation_covariance[8] = cov_orientation;

		imuMsg.angular_velocity.x = angular_velocity[0]/16.0*deg2rad;
		imuMsg.angular_velocity.y = angular_velocity[1]/16.0*deg2rad;
		imuMsg.angular_velocity.z = angular_velocity[2]/16.0*deg2rad;

		imuMsg.angular_velocity_covariance = zeros;
		imuMsg.angular_velocity_covariance[0] = cov_velocity;
		imuMsg.angular_velocity_covariance[4] = cov_velocity;
		imuMsg.angular_velocity_covariance[8] = cov_velocity;

		if(removeGravitationalAcceleration_)
		{
			imuMsg.linear_acceleration.x = linear_acceleration[0]/100.0;
			imuMsg.linear_acceleration.y = linear_acceleration[1]/100.0;
			imuMsg.linear_acceleration.z = linear_acceleration[2]/100.0;
		}
		else
		{
			imuMsg.linear_acceleration.x = acceleration[0]/100.0;
			imuMsg.linear_acceleration.y = acceleration[1]/100.0;
			imuMsg.linear_acceleration.z = acceleration[2]/100.0;
		}

		imuMsg.linear_acceleration_covariance = zeros;
		imuMsg.linear_acceleration_covariance[0] = cov_acceleration;
		imuMsg.linear_acceleration_covariance[4] = cov_acceleration;
		imuMsg.linear_acceleration_covariance[8] = cov_acceleration;

		//publish the message
		imuPub_.publish(imuMsg);
	}


	/*----------------------------------------------------------------------
	* Publish the MagneticField message.
	*--------------------------------------------------------------------*/
	
	/*if(mfPub_.getNumSubscribers()){TransformBroadcaster

		int16_t x = 0, y = 0, z = 0;
		// for IMU v2 http://www.tinkerforge.com/de/doc/Software/Bricks/IMUV2_Brick_C.html#imu-v2-brick-c-api
		
		//imu_v2_get_magnetic_field(&imu_v2, &x, &y, &z); // µT -> T
		x = magnetic_field[0] /16.0 * 1000000.0;
		y = magnetic_field[1] /16.0 * 1000000.0;
		z = magnetic_field[2] /16.0 * 1000000.0;

		sensor_msgs::MagneticField mf_msg;

		// message header
		//mf_msg.header.seq =  seq;
		mf_msg.header.stamp = ros::Time::now();
		mf_msg.header.frame_id = frameId_;

		// magnetic field from mG to T
		mf_msg.magnetic_field.x = x;
		mf_msg.magnetic_field.y = y;
		mf_msg.magnetic_field.z = z;

		boost::array<const double, 9> mfc =
		{ 0.01, 0.01, 0.01,
			0.01, 0.01, 0.01,
			0.01, 0.01, 0.01};

		mf_msg.magnetic_field_covariance = mfc;

		mfPub_.publish(mf_msg);
	}*/
	
	
	
	if(odomPub_.getNumSubscribers()){

		double angular_vel_x = angular_velocity[0]/16.0*deg2rad;
		double angular_vel_y = angular_velocity[1]/16.0*deg2rad;
		double angular_vel_z = angular_velocity[2]/16.0*deg2rad;

		double dt = (current_time - last_time).toSec();
		double delta_x = angular_vel_x * dt;
		double delta_y = angular_vel_y * dt;
		double delta_th = angular_vel_z * dt;

		x_pos += delta_x;
		y_pos += delta_y;
		th += delta_th;
		

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); // Rotation around Z (Yaw)

		//first, we'll publish the transform over tf
		static tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "map"; // Change from "base_link"

		odom_trans.transform.translation.x = x_pos;
		odom_trans.transform.translation.y = y_pos;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//odometry message header
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x_pos;
		odom.pose.pose.position.y = y_pos;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "map"; //Change from "base_link"
		odom.twist.twist.linear.x = angular_vel_x;
		odom.twist.twist.linear.y = angular_vel_y;
		odom.twist.twist.angular.z = angular_vel_z;

		//publish the message
    	odomPub_.publish(odom);
		last_time = current_time;
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brick_imu");

	ros::NodeHandle pnh("~");
	pnh.param("frame_id", frameId_, frameId_);
	int period = 100;
	std::string uid = "6JoJV7"; // Change to your UID (shown by brick viewer)
	pnh.param("period_ms", period, period);
	pnh.param("uid", uid, uid);

	pnh.param("remove_gravitational_acceleration", removeGravitationalAcceleration_, removeGravitationalAcceleration_);

	pnh.param("cov_orientation", cov_orientation, cov_orientation);
	pnh.param("cov_velocity", cov_velocity, cov_velocity);
	pnh.param("cov_acceleration", cov_acceleration, cov_acceleration);

	ros::NodeHandle nh;

	// Publish
	imuPub_ 	= nh.advertise<sensor_msgs::Imu>("imu/data", 1);
	//mfPub_ 		= nh.advertise<sensor_msgs::MagneticField>("magnetic/data", 50);
	odomPub_ 	= nh.advertise<nav_msgs::Odometry>("odom", 50);	
	current_time = ros::Time::now();
  	last_time = ros::Time::now();

	// Create IP connection
	IPConnection ipcon;
	ipcon_create(&ipcon);

	// Create device object
	IMUV2 imu;
	imu_v2_create(&imu, uid.c_str(), &ipcon);

	// Connect to brickd
	if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
		fprintf(stderr, "Could not connect to brick \n");
		return 1;
	}
	// Don't use device before ipcon is connected
	
	// Register all data callback to function cb_all_data
	imu_v2_register_callback(&imu,
	                         IMU_V2_CALLBACK_ALL_DATA,
	                         (void *)cb_all_data,
	                         NULL);

	// Set period for all data callback to 0.1s (100ms)
	// Note: The all data callback is only called every 0.1 seconds
	//       if the all data has changed since the last call!
	imu_v2_set_all_data_period(&imu, period);

	//main loop
	ros::spin();

	ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally

	return 0;
}

