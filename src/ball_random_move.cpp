/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::string node_name = "ball_random_move";
  std::string ball_vel_topic = "orange_ball/cmd_vel";
  if (argc > 1)
  {
    std::ostringstream stringStream;
    stringStream << argv[1] << "/cmd_vel";
    ball_vel_topic = stringStream.str();
  }
  srand(static_cast<unsigned>(time(NULL)));

  auto node = rclcpp::Node::make_shared(node_name);
  auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(ball_vel_topic, 10);
  rclcpp::WallRate loop_rate(0.2);

  while (rclcpp::ok())
  {
    float x_vel = (0.5 - rand() / static_cast<float>(RAND_MAX)) * 0.8;
    float y_vel = (0.5 - rand() / static_cast<float>(RAND_MAX)) * 0.8;
    RCLCPP_INFO(node->get_logger(), "(%.2f , %.2f) - > %s", x_vel, y_vel, ball_vel_topic.c_str());

    auto vel_cmd = std::make_shared<geometry_msgs::msg::Twist>();
    vel_cmd->linear.x = x_vel;
    vel_cmd->linear.y = y_vel;
    vel_cmd->linear.z = 0.0;
    vel_cmd->angular.x = 0;
    vel_cmd->angular.y = 0;
    vel_cmd->angular.z = 0;
    vel_pub->publish(*vel_cmd);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}