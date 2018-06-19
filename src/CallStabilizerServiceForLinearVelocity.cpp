#include <ros/ros.h>
#include <sys/time.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
// After idl Service is updated,
// $ cd /opt/ros/kinetic/include
// $ sudo rm -rf hrpsys_ros_bridge  # if hrpsys_ros_bridge already exists
// $ sudo scp -r hrpuser@hrp2007v:~/ros/indigo/devel/include/hrpsys_ros_bridge ./
// $ roscd zed_wrapper
// $ catkin bt
#include "hrpsys_ros_bridge/OpenHRP_StabilizerService_setSegwayParameter.h"

char *time_string()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm *tm_ = localtime(&tv.tv_sec);
  static char time[20];
  sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
  return time;
}

double now_time_second()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
    double time_sec;
    time_sec = tm_->tm_hour * 60.0 * 60.0 + tm_->tm_min * 60.0 + tm_->tm_sec + tv.tv_usec / (1000.0 * 1000.0);
    return time_sec;
}

geometry_msgs::Vector3 linear_vel_local_msg;
geometry_msgs::Vector3 acc_msg;
double v_d, omega_d;

#define V_d_max 0.3
// #define Omega_d_max 0.6
#define Omega_d_max 0.8
// #define Omega_d_max 1.0
// #define Kp 0.005
// #define Kp 0.007
//////////////////////////////////////////////////////////// #define Kp 0.01 // 手で外乱与えても振動しない  定常偏差: -0.025 [m/s]
// #define Kp 0.015 // 外乱なしだと安定(定常偏差少)だが，手で外乱を少し強く与えると振動(振幅は増えていない)  定常偏差: -0.016 [m/s]
// #define Kp 0.008
// #define Kp 0.0
//////////////////////////////////////////////////////////// #define Ki 0.002
//////////////////////////////////////////////////////////// #define Ti 5.0 // Ki = 0.002
// #define Ki 0.003 // Ki=0.002よりわずかに振動
// #define Kd 0.001
// #define Kd 0.004
// #define Kd 0.0005
// #define Kd 0.0
// #define Kd -0.001

void chatterCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  // ROS_INFO("msg->twist.twist.linear.x: %f", msg->twist.twist.linear.x);
  linear_vel_local_msg = *msg;
}

void AccSubCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  acc_msg = *msg;
}

void JoySubCallback(const sensor_msgs::JoyConstPtr& msg)
{
  v_d = V_d_max * msg->axes[1];
  omega_d = Omega_d_max * msg->axes[2];
  // std::cerr << "omega_d : " << omega_d << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "CallStabilizerServiceForLinearVelocity");
  ros::NodeHandle n;

  double before_servicecall_time_second, after_servicecall_time_second;

  // double Kp, Ki, Kd;
  // ros::param::set("/PID_gain_Kp", 0.01);
  // // ros::param::set("/PID_gain_Ki", 0.002);
  // ros::param::set("/PID_gain_Ki", 0.0);
  // // ros::param::set("/PID_gain_Kd", 0.0005);
  // ros::param::set("/PID_gain_Kd", 0.0);
  // // ros::param::set("/PID_gain_Kp", 0.0081);
  // // ros::param::set("/PID_gain_Ki", 0.005155);

  // double e_v;

  ros::Subscriber chatter_sub = n.subscribe("linear_vel_local", 1, chatterCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber acc_sub = n.subscribe("imu_linear_acc_offset_g", 1, AccSubCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber joy_sub = n.subscribe("joy", 1, JoySubCallback, ros::TransportHints().tcpNoDelay());

  // ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("linear_vel_local2", 1);

  ros::ServiceClient chatter_client = n.serviceClient<hrpsys_ros_bridge::OpenHRP_StabilizerService_setSegwayParameter>("StabilizerServiceROSBridge/setSegwayParameter");
  hrpsys_ros_bridge::OpenHRP_StabilizerService_setSegwayParameter srv;

  // // ros::Time t;
  // ros::Time old_t = ros::Time::now();
  // // double old_linear_vel_local_msg_x = linear_vel_local_msg.x;
  // double old_e_v = 0.0;
  // // double dt_linear_vel_local_msg_x;
  // // double pre_dt_linear_vel_local_msg_x = 0.0;
  // // double integral_linear_vel_local_msg_x = 0.0;
  // double integral_e_v = 0.0;
  // double differential_e_v;

  ros::Rate loop_rate(100);
  // int count = 0;
  while (ros::ok())
    {
      // ROS_INFO("count = %d", count);
      // std::cerr << "localtime: [" << time_string() << "]" << std::endl;
      ros::spinOnce();
      // ros::param::get("/PID_gain_Kp", Kp);
      // ros::param::get("/PID_gain_Ki", Ki);
      // ros::param::get("/PID_gain_Kd", Kd);
      // e_v = v_d - linear_vel_local_msg.x;
      // integral_e_v += (e_v + old_e_v) * (ros::Time::now() - old_t).toSec() * 0.5; // Integral
      // differential_e_v = (e_v - old_e_v) / (ros::Time::now() - old_t).toSec(); // Differential
      // // t = ros::Time::now();
      // // std::cerr << "(ros::Time::now() - old_t).toSec(): " << (ros::Time::now() - old_t).toSec() << std::endl;
      // // dt_linear_vel_local_msg_x = (linear_vel_local_msg.x - old_linear_vel_local_msg_x) / (ros::Time::now() - old_t).toSec(); // Differential
      // // integral_linear_vel_local_msg_x += (linear_vel_local_msg.x + old_linear_vel_local_msg_x) * (ros::Time::now() - old_t).toSec() * 0.5; // Integral
      // // if (fabs(dt_linear_vel_local_msg_x - pre_dt_linear_vel_local_msg_x) > 2.0) {
      // //   std::cerr << "hogei" << std::endl;
      // //   dt_linear_vel_local_msg_x = pre_dt_linear_vel_local_msg_x;
      // // }
      // // dt_linear_vel_local_msg_x = (linear_vel_local_msg.x - old_linear_vel_local_msg_x) / (t - old_t).toSec(); // Differential
      // // dt_linear_vel_local_msg_x = 0.85 * pre_dt_linear_vel_local_msg_x + 0.15 * dt_linear_vel_local_msg_x; // LPF
      // old_t = ros::Time::now();
      // // old_t = t;
      // // old_linear_vel_local_msg_x = linear_vel_local_msg.x;
      // old_e_v = e_v;
      // // pre_dt_linear_vel_local_msg_x = dt_linear_vel_local_msg_x;

      // // srv.request.i_param.segway_param = Kp * linear_vel_local_msg.x + Kd * dt_linear_vel_local_msg_x; // set value of segway_param
      // // srv.request.i_param.segway_param = Kp * linear_vel_local_msg.x + Ki * integral_linear_vel_local_msg_x + Kd * acc_msg.x; // set value of segway_param
      // // srv.request.i_param.segway_param = Kp * e_v + Ki * integral_e_v + Kd * acc_msg.x; // set value of segway_param
      // srv.request.i_param.segway_param = Kp * e_v + Ki * integral_e_v + Kd * differential_e_v; // set value of segway_param
      // // srv.request.i_param.segway_param = Kp * (linear_vel_local_msg.x + 1.0 / Ti * integral_linear_vel_local_msg_x) + Kd * acc_msg.x; // set value of segway_param
      srv.request.i_param.segway_av_yaw_target = omega_d; // set value of segway_av_yaw_target
      srv.request.i_param.segway_lv_x_target = v_d; // set value of segway_lv_x_target
      srv.request.i_param.segway_lv_x_act = linear_vel_local_msg.x; // set value of segway_lv_x_act

      // // plot test
      // // linear_vel_local_msg.y = dt_linear_vel_local_msg_x;
      // // linear_vel_local_msg.y = integral_linear_vel_local_msg_x;
      // // linear_vel_local_msg.y = integral_e_v;
      // linear_vel_local_msg.y = e_v;
      // linear_vel_local_msg.z = differential_e_v;

      before_servicecall_time_second = now_time_second();
      if (chatter_client.call(srv))
        {
          // // we can use srv.response (but in this case, it's void)
          // ROS_INFO("Service Call: Succeed");
        }
      else
        {
          ROS_ERROR("Service Call: Failed");
          // return 1;
        }
      after_servicecall_time_second = now_time_second();
      // std::cerr << "servicecall_second_diff = " << after_servicecall_time_second - before_servicecall_time_second << std::endl;
      if ( after_servicecall_time_second - before_servicecall_time_second > 1.5 ) {
        std::cerr << "rosservicecall costs much time !!!" << std::endl;
        std::cerr << "servicecall_second_diff = " << after_servicecall_time_second - before_servicecall_time_second << std::endl;
        std::cerr << "before_servicecall_time_second = " << before_servicecall_time_second << std::endl;
        std::cerr << "after_servicecall_time_second = " << after_servicecall_time_second << std::endl;
      }
      // chatter_pub.publish(linear_vel_local_msg);
      // ++count;
      loop_rate.sleep();
    }
}
