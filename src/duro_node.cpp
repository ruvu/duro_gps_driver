#include <duro_gps_driver/duro_gps_driver.hpp>

/**
 * STEPS:
 *
 *    1. Initialize ROS nodehandlers
 *    2. Initialize driver (open sockets, initialize ROS ecosystem)
 *    3. If successful, poll messages from sensor and spin ROS queue
 *    4. On finish, close sockets
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "duro");
  ros::NodeHandle nh;						///< This is for global access
  ros::NodeHandle private_nh("~");			///< Use private_nh for local parameters

  std::string tcp_ip_addr;
  int tcp_ip_port;

  if (!private_nh.getParam("address", tcp_ip_addr))
  {
	  ROS_ERROR("No address is specified, exiting");
	  return -2;
  }
  if (!private_nh.getParam("port", tcp_ip_port))
  {
	  ROS_ERROR("No port specified, exiting");
	  return -2;
  }
  duro::DuroDriver driver(nh, tcp_ip_addr, tcp_ip_port);
  if (driver.initialize())
  {
	  ROS_INFO("Successfully initialized DURO GPS driver");
	  while (ros::ok())
	  {
		  // You can process its return if you want
		  driver.poll_sockets();
		  ros::spinOnce();
	  }
	  driver.close_socket();
	  return 0;
  }
  //ros::Rate loop_rate(10);
  return -1;
}
