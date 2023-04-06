#ifndef ROBOTEQ_DIFF_DRIVER__ROBOTEQ_DIFF_DRIVER_HPP_
#define ROBOTEQ_DIFF_DRIVER__ROBOTEQ_DIFF_DRIVER_HPP_

#include <math.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
  public:
  explicit Roboteq(); //(nodeOptions options?)
  ~Roboteq();

  private:
  // class atributes
  //rclcpp::Node::SharedPtr nh{};

  uint32_t starttime{};
  uint32_t hstimer{};
  uint32_t mstimer{};
  uint32_t lstimer{};
  rclcpp::TimerBase::SharedPtr timer_;

  // buffer for reading encoder counts
  unsigned int odom_idx{};
  char odom_buf[24]{};

  // toss out initial encoder readings
  char odom_encoder_toss{};

  int32_t odom_encoder_left{};
  int32_t odom_encoder_right{};

  float odom_x{};
  float odom_y{};
  float odom_yaw{};
  float odom_last_x{};
  float odom_last_y{};
  float odom_last_yaw{};

  uint32_t odom_last_time{};


  // settings
  bool pub_odom_tf{};
  std::string odom_frame{};
  std::string base_frame{};
  std::string cmdvel_topic{};
  std::string odom_topic{};
  std::string port{};
  int baud{};
  bool open_loop{};
  double wheel_circumference{};
  double track_width{};
  int encoder_ppr{};
  int encoder_cpr{};
  double max_amps{};
  int max_rpm{};
  // Test different odom msg memory
  //nav_msgs::msg::Odometry odom_msg{};
  nav_msgs::msg::Odometry odom_msg{};
  //geometry_msgs::msg::Twist twist_msg{};



    //
  // cmd_vel subscriber
  //
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();
 


  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream(); 
  void odom_loop();
  //void odom_hs_run();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish();
  void connect();

  void update_parameters();
  int run();

  //subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

  //publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;



};

}





#endif
