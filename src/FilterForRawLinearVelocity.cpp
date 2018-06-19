#include <ros/ros.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sl/Camera.hpp>  // /usr/local/zed/include/sl/Camera.hpp

char *time_string()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm *tm_ = localtime(&tv.tv_sec);
  static char time[20];
  sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
  return time;
}

geometry_msgs::Vector3 optical_linear_vel_local_msg;
geometry_msgs::Vector3 imu_linear_acc_msg;
geometry_msgs::Vector3 linear_vel_local_msg;
geometry_msgs::Vector3 pre_linear_vel_local_msg;

sl::Orientation quat_imu;
sl::Translation g_world = {0.0, 0.0, -9.8};
sl::Translation g_local;

// #define Alpha 0.8
// #define Alpha 0.95
#define Alpha 0.98
// #define Alpha 0.55

void zedOdomSubCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // ROS_INFO("msg->twist.twist.linear.x: %f", msg->twist.twist.linear.x);
  optical_linear_vel_local_msg = msg->twist.twist.linear;
}

void hrp2ImuSubCallback(const sensor_msgs::ImuConstPtr& msg)
{
  quat_imu.x = msg->orientation.x;
  quat_imu.y = msg->orientation.y;
  quat_imu.z = msg->orientation.z;
  quat_imu.w = msg->orientation.w;
  g_local = g_world * quat_imu;
  // std::cerr << "g_local: " << g_local << std::endl;
  imu_linear_acc_msg.x = msg->linear_acceleration.x - g_local(0);
  imu_linear_acc_msg.y = msg->linear_acceleration.y - g_local(1);
  imu_linear_acc_msg.z = msg->linear_acceleration.z + g_local(2);
  // std::cerr << "imu_linear_acc_msg.x: " << imu_linear_acc_msg.x << std::endl;
  // std::cerr << "imu_linear_acc_msg.y: " << imu_linear_acc_msg.y << std::endl;
  // std::cerr << "imu_linear_acc_msg.z: " << imu_linear_acc_msg.z << std::endl;
  // std::cerr << "//////////////////////" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FilterForRawLinearVelocity");
  ros::NodeHandle n;

  ros::Subscriber zed_odom_sub = n.subscribe("zed/odom", 1, zedOdomSubCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber hrp2_imu_sub = n.subscribe("imu", 1, hrp2ImuSubCallback, ros::TransportHints().tcpNoDelay());

  ros::Publisher filtered_vel_pub = n.advertise<geometry_msgs::Vector3>("linear_vel_local", 1);
  ros::Publisher imu_acc_pub = n.advertise<geometry_msgs::Vector3>("imu_linear_acc_offset_g", 1);

  ros::Time pre_t = ros::Time::now();
  double dt;
  // double old_linear_vel_local_msg_x = linear_vel_local_msg.x;
  // double dt_linear_vel_local_msg_x;
  // double pre_dt_linear_vel_local_msg_x = 0.0;

  ros::Rate loop_rate(100);
  // int count = 0;
  while (ros::ok())
    {
      // ROS_INFO("count = %d", count);
      // std::cerr << "localtime: [" << time_string() << "]" << std::endl;
      ros::spinOnce();
      dt = (ros::Time::now() - pre_t).toSec();

      // Complementary Filter
      linear_vel_local_msg.x = Alpha * (pre_linear_vel_local_msg.x + imu_linear_acc_msg.x * dt) + (1 - Alpha) * optical_linear_vel_local_msg.x;

      // Original Filter to Remove Impulse
      // if ((fabs(linear_vel_local_msg.x - pre_linear_vel_local_msg.x) > 0.3) || (fabs(linear_vel_local_msg.y - pre_linear_vel_local_msg.y) > 0.3) || (fabs(linear_vel_local_msg.z - pre_linear_vel_local_msg.z) > 0.3))
      if (fabs((pre_linear_vel_local_msg.x + imu_linear_acc_msg.x * dt) - optical_linear_vel_local_msg.x) > 8.0) {
        std::cerr << "Ijoukensyutsu2" << std::endl;
        linear_vel_local_msg.x = pre_linear_vel_local_msg.x + imu_linear_acc_msg.x * dt;
      }

      pre_t = ros::Time::now();
      pre_linear_vel_local_msg = linear_vel_local_msg;

      filtered_vel_pub.publish(linear_vel_local_msg);
      imu_acc_pub.publish(imu_linear_acc_msg);

      // ++count;
      loop_rate.sleep();
    }
}
