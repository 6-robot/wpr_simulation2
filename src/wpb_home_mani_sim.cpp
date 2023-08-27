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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

static float nMinHeight = 0.493;
static float nMaxHeight = 1.036;
static float nBottomHeight = 0.32;

class WPB_Home_Mani_Sim_Node : public rclcpp::Node
{
public:
    WPB_Home_Mani_Sim_Node() : Node("wpb_home_mani_node")
    {
        // 初始化关节控制消息包
        pub_msg_.data.resize(6);
        for(int i=0;i<6;i++)
        {
            pub_msg_.data[i] = 0;
            target_position_[i] = 0;
        }

        // 创建订阅器，订阅/wpb_home/mani_ctrl话题
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/wpb_home/mani_ctrl", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            processJointState(msg);
            });

        // 创建发布器，发布/manipulator_controller/commands话题
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/manipulator_controller/commands", 10);
        // 定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
        [this]() {
          publishMessage();
        });
    }

    void ManiGripper(float inGripperValue)
    {
        float fGripperAngle = asin((inGripperValue - 0.025)*5);
        target_position_[2] = fGripperAngle;
        target_position_[3] = -fGripperAngle;
        target_position_[4] = -fGripperAngle;
        target_position_[5] = fGripperAngle;
    }
    void ManiHeight(float inHeight)
    {
        if(inHeight >= nMinHeight)
        {
            target_position_[1] = 1.5707963;
            float fLiftPos = inHeight - nBottomHeight;
            if(inHeight >nMaxHeight )
            {
                fLiftPos = nMaxHeight - nBottomHeight;
            }
            target_position_[0] = fLiftPos;
        }
        else
        {
            if(inHeight - nBottomHeight > 0)
                target_position_[0] = inHeight - nBottomHeight;
            else
                target_position_[0] = 0;
            target_position_[1] = 0;
        }

        
    }

    void publishMessage()
    {
        if(target_position_[0] > pub_msg_.data[0])
            pub_msg_.data[0] += 0.003;
        if(target_position_[0] < pub_msg_.data[0])
            pub_msg_.data[0] -= 0.003;
        if(target_position_[1] > pub_msg_.data[1])
            pub_msg_.data[1] += 0.012;
        if(target_position_[1] < pub_msg_.data[1])
            pub_msg_.data[1] -= 0.012;
        for(int i=2;i<6;i++)
        {
            if(target_position_[i] > pub_msg_.data[i])
                pub_msg_.data[i] += 0.02;
            if(target_position_[i] < pub_msg_.data[i])
                pub_msg_.data[i] -= 0.02;
        }
        
        // 发布Float64MultiArray消息到/manipulator_controller/commands话题
        publisher_->publish(pub_msg_);
    }

private:
    void processJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 提取名称为"lift"和"gripper"的位置
        std::vector<double> positions;
        for (size_t i = 0; i < msg->name.size(); ++i) 
        {
            if (msg->name[i] == "lift") 
            {
                ManiHeight(msg->position[i]);
            }
            if (msg->name[i] == "gripper") 
            {
                ManiGripper(msg->position[i]);
            }
        }

        // 发布Float64MultiArray消息到/manipulator_controller/commands话题
        publisher_->publish(pub_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std_msgs::msg::Float64MultiArray pub_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    float target_position_[6];
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WPB_Home_Mani_Sim_Node>());
  rclcpp::shutdown();
  return 0;
}