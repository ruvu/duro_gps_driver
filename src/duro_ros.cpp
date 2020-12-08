#include "duro_ros.h"

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


DuroROS::DuroROS():
    socket_desc_(-1)
{
    ros::NodeHandle nh;
    nav_fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("fix", 100, true);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("mag", 100);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 100);
    status_flag_pub_ = nh.advertise<std_msgs::UInt8>("status_flag", 100, true);
    status_string_pub_ = nh.advertise<std_msgs::String>("status_string", 100, true);

    ros::NodeHandle nh_private;
    nh_private.param<std::string>("ip_address", tcp_ip_address_, "192.168.0.222");
    nh_private.param<int>("ip_port", tcp_ip_port_, 55555);
    nh_private.param<std::string>("gps_receiver_frame_id", gps_receiver_frame_id_, "gps_receiver_frame");
    nh_private.param<std::string>("imu_frame_id", imu_frame_id_, gps_receiver_frame_id_);

    ROS_INFO_STREAM("Using ip:port " << tcp_ip_address_ << ":" << tcp_ip_port_);

    setup_socket();
    sbp_state_init(&s_);
    sbp_state_set_io_context(&s_, this);
    sbp_register_callback(&s_, SBP_MSG_POS_LLH, &pos_cb_forwarder, this, &pos_ll_cov_callback_node_);
    sbp_register_callback(&s_, SBP_MSG_IMU_RAW, &imu_cb_forwarder, this, &imu_callback_node_);
    sbp_register_callback(&s_, SBP_MSG_MAG_RAW, &mag_cb_forwarder, this, &mag_callback_node_);
}

DuroROS::~DuroROS()
{
    close(socket_desc_);
}

void DuroROS::setup_socket()
{
    socket_desc_= socket(AF_INET, SOCK_STREAM, 0);
    if (socket_desc_== -1)
    {
        ROS_ERROR("Could not create socket");
    }

    memset(&server_, '0', sizeof(server_));
    server_.sin_addr.s_addr = inet_addr(tcp_ip_address_.c_str());
    server_.sin_family = AF_INET;
    server_.sin_port = htons(tcp_ip_port_);

    if (connect(socket_desc_, (struct sockaddr *)&server_, sizeof(server_)) < 0)
    {
        ROS_ERROR("Connection error");
    }
}

s32 DuroROS::socket_read(u8 *buff, u32 n)
{
    s32 result;

    result = read(socket_desc_, buff, n);

    return result;
}

void DuroROS::tick()
{
    sbp_process(&s_, &socket_read_forwarder);
}

void DuroROS::pos_ll_cov_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[])
{
    (void)msg;
    msg_pos_llh_t *lat_long_msg = (msg_pos_llh_t *)msg;

    std_msgs::String status_msg;

    int ins_mode = (lat_long_msg->flags & INS_MODE_MASK) >> INS_MODE_POSITION;
    int fix_mode = (lat_long_msg->flags & FIX_MODE_MASK) >> FIX_MODE_POSITION;

    if ( fix_mode != fix_modes::INVALID )
    {
        sensor_msgs::NavSatFix fix;
        fix.header.stamp = ros::Time::now();
        fix.header.frame_id = gps_receiver_frame_id_;
        fix.latitude = lat_long_msg->lat;
        fix.longitude = lat_long_msg->lon;
        fix.altitude = lat_long_msg->height;
        fix.position_covariance[0] = lat_long_msg->h_accuracy * lat_long_msg->h_accuracy * 1e-6;
        fix.position_covariance[4] = lat_long_msg->h_accuracy * lat_long_msg->h_accuracy * 1e-6;
        fix.position_covariance[8] = lat_long_msg->v_accuracy * lat_long_msg->v_accuracy * 1e-6;

        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        // We don't know the current service provider from the fix message, but we know
        // Swift Duro supports all of them, so let's assume we see some of each system's
        // satellites for lack of a better implementation.
        //TODO(rokus): check which satellite systems are actually in use at the moment.
        fix.status.service = sensor_msgs::NavSatStatus::SERVICE_COMPASS +
                             sensor_msgs::NavSatStatus::SERVICE_GALILEO +
                             sensor_msgs::NavSatStatus::SERVICE_GLONASS +
                             sensor_msgs::NavSatStatus::SERVICE_GPS;

        // TODO(rokus): add custom message for Swift-specific fix modes.
        switch (fix_mode)
        {
            case fix_modes::INVALID:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                status_msg.data = "Invalid";
                break;
            case fix_modes::SINGLE_POINT_POSITION:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                status_msg.data = "Single Point Position (SPP)";
                break;
            case fix_modes::DIFFERENTIAL_GNSS:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                status_msg.data = "Differential GNSS (DGNSS)";
                break;
            case fix_modes::FLOAT_RTK:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                status_msg.data = "Float RTK";
                break;
            case fix_modes::FIXED_RTK:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                status_msg.data = "Fixed RTK";
                break;
            case fix_modes::DEAD_RECKONING:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                status_msg.data = "Dead Reckoning (DR)";
                break;
            case fix_modes::SBAS_POSITION:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
                status_msg.data = "SBAS Position";
                break;
            default:
                fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                ROS_WARN_STREAM("Acquired a fix with a mode that's not implemented. You are likely"
                                "using an unsupported version of libsbp.");
                status_msg.data = "Not implemented";
                break;
        }
        nav_fix_pub_.publish(fix);
    }
    status_string_pub_.publish(status_msg);
}

void DuroROS::imu_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[])
{
    (void)msg;
    msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
    sensor_msgs::Imu imu_msg_ros;
    imu_msg_ros.header.stamp = ros::Time::now();
    imu_msg_ros.header.frame_id = imu_frame_id_;

    imu_msg_ros.linear_acceleration.x = imumsg->acc_x;
    imu_msg_ros.linear_acceleration.y = imumsg->acc_y;
    imu_msg_ros.linear_acceleration.z = imumsg->acc_z;
    // Covariance is unknown, so we don't fill the matrix

    imu_msg_ros.angular_velocity.x = imumsg->gyr_x;
    imu_msg_ros.angular_velocity.y = imumsg->gyr_y;
    imu_msg_ros.angular_velocity.z = imumsg->gyr_z;
    // Covariance is unknown, so we don't fill the matrix

    imu_pub_.publish(imu_msg_ros);
}

void DuroROS::mag_callback(u16 /*sender_id*/, u8 /*len*/, u8 msg[])
{
    (void)msg;
    msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
    sensor_msgs::MagneticField mag_ros_msg;
    mag_ros_msg.header.stamp = ros::Time::now();
    mag_ros_msg.header.frame_id = imu_frame_id_;

    mag_ros_msg.magnetic_field.x = magmsg->mag_x * 1e-6;
    mag_ros_msg.magnetic_field.y = magmsg->mag_y * 1e-6;
    mag_ros_msg.magnetic_field.z = magmsg->mag_z * 1e-6;
    mag_pub_.publish(mag_ros_msg);
}
