/*
 * duro_gps_driver.cpp
 *
 *  Created on: Apr 15, 2020
 *      Author: kyberszittya
 */

#include <duro_gps_driver/duro_gps_driver.hpp>


namespace duro
{

DuroDriver::DuroDriver(ros::NodeHandle& nh, const std::string tcp_ip_addr,
			const int tcp_ip_port):
				nh(nh), tcp_ip_addr(tcp_ip_addr), tcp_ip_port(tcp_ip_port),
				socket_desc(-1){}


bool DuroDriver::setup_socket()
{
	struct sockaddr_in server;
	socket_desc = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_desc == -1)
	{
		ROS_ERROR("Could not create socket");
		return false;
	}

	memset(&server, '0', sizeof(server));
	server.sin_addr.s_addr = inet_addr(tcp_ip_addr.c_str());
	server.sin_family = AF_INET;
	server.sin_port = htons(55555);

	if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		ROS_ERROR("Connection error");
		return false;
	}
	return true;

}

void DuroDriver::heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void)sender_id, (void)len, (void)msg, (void)context;
	//fprintf(stdout, "%s\n", __FUNCTION__);
	/*for(int i = 0; i < len; i++)
	printf("%d ", msg[i]);
	printf("len: %d\n", len);*/
}

/*
 * Static C-type callback to fetch pose
 */
void DuroDriver::pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void)sender_id, (void)len, (void)msg;
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
	// nav fix (latlon) message over ROS
	// TODO: GPS sensors ALWAYS support timestamps. Give the possibility to use timestamps received from sensors.
	driver->fix.header.stamp = ros::Time::now();
	if (latlonmsg->lat != 0.0)
	{
		driver->fix.latitude = latlonmsg->lat;
		driver->fix.longitude = latlonmsg->lon;
		driver->nav_fix_pub.publish(driver->fix);
		double x = 0, y = 0;
		driver->coordinate_transition.LatLonToUTMXY(latlonmsg->lat, latlonmsg->lon, x, y);
		driver->odom.header.stamp = ros::Time::now();
		driver->odom.pose.pose.position.x = x;
		driver->odom.pose.pose.position.y = y;
		driver->odom.pose.pose.position.z = latlonmsg->height;
		driver->odom_pub.publish(driver->odom);						///< We supply faux odometry. Because why the hell not.
		// The important part: pose support
		driver->pose_msg.header.stamp = ros::Time::now();
		driver->pose_msg.pose.position.x = x;
		driver->pose_msg.pose.position.y = y;
		driver->pose_msg.pose.position.z = latlonmsg->height;
		driver->pose_pub.publish(driver->pose_msg);					///< Pose is published
		std_msgs::UInt8 flags;
		flags.data = latlonmsg->flags;
		driver->status_flag_pub.publish(flags); // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3: Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position
		std_msgs::String stflags;

		switch (latlonmsg->flags)
		{
		case 9: // 1 // TODO: check
		  stflags.data = "Single Point Position (SPP)";
		  break;
		case 10: // 2
		  stflags.data = "Differential GNSS (DGNSS)";
		  break;
		case 11: // 3
		  stflags.data = "Float RTK";
		  break;
		case 12: // 4
		  stflags.data = "Fixed RTK";
		  break;
		case 13:
		  stflags.data = "Dead Reckoning (DR)";
		  break;
		case 14:
		  stflags.data = "SBAS Position";
		  break;
		default:
		  stflags.data = "Invalid";
		  break;
		}

		driver->status_stri_pub.publish(stflags);
	}
}

/*
 * Static C-type callback to fetch orientation in Quaternion
 */
void DuroDriver::orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	// enable MSG ID 544 in swift console
	// the MSG ID comes from eg #define SBP_MSG_ORIENT_QUAT 0x0220 --> 544
	(void)sender_id, (void)len, (void)msg;
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	msg_orient_quat_t *orimsg = (msg_orient_quat_t *)msg;

	double w = orimsg->w * pow(2, -31);
	double x = orimsg->x * pow(2, -31);
	double y = orimsg->y * pow(2, -31);
	double z = orimsg->z * pow(2, -31);
	tf2::Quaternion tf_orig(x, y, z, w);
	tf2::Quaternion tf_rot, tf_aligned;
	// Check yo math bro
	tf_rot.setRPY(0.0, 0.0, -M_PI_2); // left-handed / right handed rotation
	tf_aligned = tf_rot * tf_orig;    // left-handed / right handed rotation
	driver->pose_msg.pose.orientation.w = tf_aligned.w() * -1;
	driver->pose_msg.pose.orientation.x = tf_aligned.y();      // left-handerd / right handed orientation
	driver->pose_msg.pose.orientation.y = tf_aligned.x() * -1; // left-handerd / right handed orientation
	driver->pose_msg.pose.orientation.z = tf_aligned.z();      // left-handerd / right handed orientation
	driver->pose_msg.header.frame_id = "duro";
}

/*
 * Static C-type callback to fetch orientation in RPY
 */
void DuroDriver::orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	// enable MSG ID 545 in swift console
	(void)sender_id, (void)len, (void)msg;
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	msg_orient_euler_t *orimsg = (msg_orient_euler_t *)msg;
	geometry_msgs::Vector3 eulervect;
	eulervect.x = orimsg->roll / 57292374.; // 57292374: raw > microdegrees > rad constant
	eulervect.y = orimsg->pitch / 57292374.;
	eulervect.z = orimsg->yaw / 57292374.;
	driver->euler_pub.publish(eulervect);
}


/*
 * Static C-type callback for TIME stamp
 */
void DuroDriver::time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void)sender_id, (void)len, (void)msg, (void)context;
	msg_gps_time_t *timemsg = (msg_gps_time_t *)msg;
	//printf("%d\n", timemsg->tow);
}

void DuroDriver::imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void)sender_id, (void)len, (void)msg, (void)context;
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
	geometry_msgs::Vector3 imu_ros_msg;
	imu_ros_msg.x = imumsg->acc_x;
	imu_ros_msg.y = imumsg->acc_y;
	imu_ros_msg.z = imumsg->acc_z;
	driver->imu_pub.publish(imu_ros_msg);
	geometry_msgs::Vector3 gyro_ros_msg;
	gyro_ros_msg.x = imumsg->gyr_x; // Angular rate around IMU frame X axis
	gyro_ros_msg.y = imumsg->gyr_y;
	gyro_ros_msg.z = imumsg->gyr_z;
	driver->gyro_pub.publish(gyro_ros_msg);
}

void DuroDriver::mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void)sender_id, (void)len, (void)msg;
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
	sensor_msgs::MagneticField mag_ros_msg;       // is geometry_msgs::Vector3 better?
	mag_ros_msg.magnetic_field.x = magmsg->mag_x; // Magnetic field in the body frame X axis [microteslas]
	mag_ros_msg.magnetic_field.y = magmsg->mag_y;
	mag_ros_msg.magnetic_field.z = magmsg->mag_z;
	driver->mag_pub.publish(mag_ros_msg);
}

s32 DuroDriver::socket_read(u8 *buff, u32 n, void *context)
{
	DuroDriver* driver = reinterpret_cast<DuroDriver*>(context);
	s32 result;

	result = read(driver->socket_desc, buff, n);
	return result;
}

void DuroDriver::initializeSbpCallbacks()
{
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &DuroDriver::heartbeat_callback, this, &heartbeat_callback_node);
	sbp_register_callback(&s, SBP_MSG_POS_LLH, &DuroDriver::pos_ll_callback, this, &pos_ll_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_QUAT, &DuroDriver::orientation_callback, this, &orientation_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &DuroDriver::orientation_euler_callback, this, &orientation_euler_callback_node);
	sbp_register_callback(&s, SBP_MSG_GPS_TIME, &DuroDriver::time_callback, this, &time_callback_node);
	sbp_register_callback(&s, SBP_MSG_IMU_RAW, &DuroDriver::imu_callback, this, &imu_callback_node);
	sbp_register_callback(&s, SBP_MSG_MAG_RAW, &DuroDriver::mag_callback, this, &mag_callback_node);
	sbp_state_init(&s);
}

void DuroDriver::initializeRos()
{
	chatter_pub_st = nh.advertise<std_msgs::String>("gps/duro/utmzone", 100);
	odom_pub = nh.advertise<nav_msgs::Odometry>("gps/duro/odom", 100);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gps/duro/current_pose", 100);
	nav_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/duro/fix", 100);
	mag_pub = nh.advertise<sensor_msgs::MagneticField>("gps/duro/mag", 100);
	imu_pub = nh.advertise<geometry_msgs::Vector3>("gps/duro/imu", 100);
	gyro_pub = nh.advertise<geometry_msgs::Vector3>("gps/duro/gyro", 100);
	euler_pub = nh.advertise<geometry_msgs::Vector3>("gps/duro/rollpitchyaw", 100);
	status_flag_pub = nh.advertise<std_msgs::UInt8>("gps/duro/status_flag", 100);
	status_stri_pub = nh.advertise<std_msgs::String>("gps/duro/status_string", 100);
	tmp_pub = nh.advertise<std_msgs::Float64>("gps/duro/tmp", 100);
}

bool DuroDriver::initialize()
{

	// Setup SBP sensor connection
	if (!setup_socket())
	{
		ROS_ERROR("Unable to steup DURO, exiting");
		return false;
	}
	ROS_INFO_STREAM("Initialized with parameters: " << tcp_ip_addr << '\t' << tcp_ip_port);
	/*
	sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);
	sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_ll_callback, NULL, &pos_ll_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_QUAT, &orientation_callback, NULL, &orientation_callback_node);
	sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orientation_euler_callback, NULL, &orientation_euler_callback_node);
	sbp_register_callback(&s, SBP_MSG_GPS_TIME, &time_callback, NULL, &time_callback_node);
	sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_callback, NULL, &imu_callback_node);
	sbp_register_callback(&s, SBP_MSG_MAG_RAW, &mag_callback, NULL, &mag_callback_node);
	*/
	// This is why we love C-type callbacks.
	initializeSbpCallbacks();
	// The initialize the ROS ecosystem
	initializeRos();
	return true;
}

void DuroDriver::close_socket()
{
	close(socket_desc);
}

s8 DuroDriver::poll_sockets()
{
	return sbp_process(&s, &DuroDriver::socket_read);
}

}
