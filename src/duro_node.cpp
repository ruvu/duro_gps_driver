// ros headers
#include "duro_ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "duro_driver");
  DuroROS duro_ros;
  while (ros::ok())
  {
      duro_ros.tick();
      ros::spinOnce();
  }
  return 0;
}

