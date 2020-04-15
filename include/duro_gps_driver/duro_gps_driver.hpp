/*
 * duro_gps_drive.hpp
 *
 *  Created on: Apr 15, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_DURO_GPS_DRIVER_DURO_GPS_DRIVER_HPP_
#define INCLUDE_DURO_GPS_DRIVER_DURO_GPS_DRIVER_HPP_


// ros headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
// libsbp - Swift Binary Protocol library headers
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>
// include folder headers
#include <duro_gps_driver/UTM.h>

namespace duro
{

class DuroDriver
{
protected:
	ros::NodeHandle nh;

	sensor_msgs::NavSatFix fix;
	ros::Publisher nav_fix_pub;
	ros::Publisher odom_pub;
	ros::Publisher chatter_pub_st;
	ros::Publisher imu_pub;
	ros::Publisher mag_pub;
	ros::Publisher gyro_pub;
	ros::Publisher euler_pub;
	ros::Publisher pose_pub;
	ros::Publisher tmp_pub;
	ros::Publisher status_flag_pub;
	ros::Publisher status_stri_pub;

	const std::string tcp_ip_addr;
	const int tcp_ip_port;
	int opt;
	int result = 0;
	sbp_state_t s;

	sbp_msg_callbacks_node_t heartbeat_callback_node;
	sbp_msg_callbacks_node_t pos_ll_callback_node;
	sbp_msg_callbacks_node_t orientation_callback_node;
	sbp_msg_callbacks_node_t orientation_euler_callback_node;
	sbp_msg_callbacks_node_t time_callback_node;
	sbp_msg_callbacks_node_t imu_callback_node;
	sbp_msg_callbacks_node_t mag_callback_node;
	nav_msgs::Odometry odom;
	geometry_msgs::PoseStamped pose_msg;
	CoordinateTransition coordinate_transition;
	int socket_desc;


public:
	DuroDriver(ros::NodeHandle& nh, const std::string tcp_ip_addr,
			const int tcp_ip_port);


	/**
	 * @group: SBP callback
	 * @brief: Heartbeat callback between sensor-host
	 */
	static void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/*
	 * @group: SBP callback
	 * @brief: GPS Position callback, which publishes pose
	 */
	static void pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/*
	 * @group: SBP callback
	 * @brief: Callback to fetch orientation (Quaternion)
	 */
	static void orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/*
	 * @group: SBP callback
	 * @brief: Callback to fetch orientation (RPY)
	 */
	static void orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	static void time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/**
	 * @group: SBP callback
	 * @brief: Callback to fetch IMU information and publish it on ROS communication graph
	 */
	static void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/**
	 * @group: SBP callback
	 * @brief: Callback to fetch magnetometer information and publish it on ROS communication graph
	 */
	static void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context);

	/**
	 * @brief: Read incoming socket from GPS sensor
	 */
	static s32 socket_read(u8 *buff, u32 n, void *context);


	bool setup_socket();


	/**
	 * @brief: initialize SBP related callbacks
	 */
	void initializeSbpCallbacks();

	/**
	 * @brief: initialize ROS pub/sub
	 */
	void initializeRos();

	/**
	 * @brief: initialize driver, read parameters
	 */
	bool initialize();

	/**
	 * @brief: close socket
	 */
	void close_socket();

	/**
	 * @brief: poll sockets
	 */
	s8 poll_sockets();
};

}

#endif /* INCLUDE_DURO_GPS_DRIVER_DURO_GPS_DRIVER_HPP_ */
