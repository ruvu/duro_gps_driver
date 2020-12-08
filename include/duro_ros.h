#ifndef DURO_ROS
#define DURO_ROS

// ros headers
#include <ros/ros.h>

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
#include <libsbp/imu.h>
#include <libsbp/mag.h>

// Bitmasks for the flags field of the pos_ll_cov message as defined in the pdf that
// can be found here: http://swift-nav.github.io/libsbp/

// first three bits are fix mode
const int FIX_MODE_POSITION = 0;
const u8 FIX_MODE_MASK = 7;

namespace fix_modes
{
enum FIX_MODE
{
    INVALID = 0,
    SINGLE_POINT_POSITION,
    DIFFERENTIAL_GNSS,
    FLOAT_RTK,
    FIXED_RTK,
    DEAD_RECKONING,
    SBAS_POSITION,
    NOT_IMPLEMENTED
};
}

// next two bits are internal navigation system mode
const int INS_MODE_POSITION = 3;
const u8 INS_MODE_MASK = 3 << INS_MODE_POSITION;

namespace ins_modes
{
enum INS_MODE
{
    NONE = 0,
    INS_USED,
    NOT_IMPLEMENTED
};
}

class DuroROS
{
private:
    ros::Publisher nav_fix_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher status_flag_pub_;
    ros::Publisher status_string_pub_;

    int tcp_ip_port_;
    std::string tcp_ip_address_;
    std::string gps_receiver_frame_id_;
    std::string imu_frame_id_;

    void setup_socket();

    void close_socket();

    int socket_desc_;

    struct sockaddr_in server_;

    sbp_state_t s_;

    sbp_msg_callbacks_node_t pos_ll_cov_callback_node_;
    sbp_msg_callbacks_node_t imu_callback_node_;
    sbp_msg_callbacks_node_t mag_callback_node_;
public:
    DuroROS();
    ~DuroROS();

    void pos_ll_cov_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[]);

    void imu_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[]);

    void mag_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[]);

    void tick();

    s32 socket_read(u8 *buff, u32 n);
};

void pos_cb_forwarder(u16 sender_id, u8 len, u8 msg[], void *context)
{
  static_cast<DuroROS*>(context)->pos_ll_cov_callback(sender_id, len, msg);
}

void imu_cb_forwarder(u16 sender_id, u8 len, u8 msg[], void *context)
{
  static_cast<DuroROS*>(context)->imu_callback(sender_id, len, msg);
}

void mag_cb_forwarder(u16 sender_id, u8 len, u8 msg[], void *context)
{
  static_cast<DuroROS*>(context)->mag_callback(sender_id, len, msg);
}

s32 socket_read_forwarder(u8 *buff, u32 n, void *context)
{
  return static_cast<DuroROS*>(context)->socket_read(buff, n);
}

#endif
