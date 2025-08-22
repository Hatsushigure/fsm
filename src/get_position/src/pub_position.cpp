#include "simulator_lcmt.hpp"
#include <chrono>
#include <inttypes.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "self_interface/msg/detail/lcmt_position__struct.hpp"
#include "self_interface/msg/lcmt_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sys/select.h>
#include <unistd.h>// 系统调用（如sleep）

simulator_lcmt simulator_state_current;//定义在这是为了让两个类都能用到
class Handler {
  public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                       const simulator_lcmt *msg) {

        simulator_state_current = *msg;
    }
};

class Dog_Position : public rclcpp::Node {
  public:
    Dog_Position(std::string node_name = "get_dog_Position") : Node(node_name) {
        pos_command_publisher_ = this->create_publisher<self_interface::msg::LCMTPosition>("gazebo_position", 10);
    }

    void timer_callback(simulator_lcmt lcm_msg) {
        self_interface::msg::LCMTPosition lcmt_msg;
        //机身坐标下的xyz速度
        lcmt_msg.cydog_vel.x = simulator_state_current.vb[0];
        lcmt_msg.cydog_vel.y = simulator_state_current.vb[1];
        lcmt_msg.cydog_vel.z = simulator_state_current.vb[2];

        //翻滚角 俯仰角 偏航角
        lcmt_msg.cydog_rpy.x = simulator_state_current.rpy[0];
        lcmt_msg.cydog_rpy.y = simulator_state_current.rpy[1];
        lcmt_msg.cydog_rpy.z = simulator_state_current.rpy[2];

        //机身朝向四元数
        lcmt_msg.facing.x = simulator_state_current.quat[1];
        lcmt_msg.facing.y = simulator_state_current.quat[2];
        lcmt_msg.facing.z = simulator_state_current.quat[3];
        lcmt_msg.facing.w = simulator_state_current.quat[0];
        //实际时间戳
        lcmt_msg.timestamps = simulator_state_current.timesteps;
        lcmt_msg.time = simulator_state_current.time;
        //机身角速度
        lcmt_msg.cydog_angle_vel.x = simulator_state_current.omega[0];
        lcmt_msg.cydog_angle_vel.y = simulator_state_current.omega[1];
        lcmt_msg.cydog_angle_vel.z = simulator_state_current.omega[2];

       //机身位置
        lcmt_msg.cydog_pose.position.x = simulator_state_current.p[0];
        lcmt_msg.cydog_pose.position.y = simulator_state_current.p[1];
        lcmt_msg.cydog_pose.position.z = simulator_state_current.p[2];
        pos_command_publisher_->publish(lcmt_msg);
        // RCLCPP_INFO(this->get_logger(), "LCMT message sent");
        // RCLCPP_INFO(this->get_logger(),"yaw:%lf",lcmt_msg.cydog_rpy.z);//%lf是double
    }

  private:
    rclcpp::Publisher<self_interface::msg::LCMTPosition>::SharedPtr pos_command_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    if (rclcpp::ok()) {
        lcm::LCM lcm;
        if (!lcm.good())
            return 1;

        auto node = std::make_shared<Dog_Position>("dog_Position");

        Handler handlerObject;
        lcm.subscribe("simulator_state", &Handler::handleMessage, &handlerObject);

        while (0 == lcm.handle()) {
            node->timer_callback(simulator_state_current);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
            rclcpp::spin_some(node);
        };

        rclcpp::shutdown();
    }


    return 0;
}