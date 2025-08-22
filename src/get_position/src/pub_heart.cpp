#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
class Dog_Heart : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Dog_Heart(std::string name) : Node(name)
    {
        // 创建心跳发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::Float64>("heart", 10);
        // 创建定时器，100ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Dog_Heart::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::Float64 message;
        message.data = 0;
        // RCLCPP_INFO(this->get_logger(), "Publishing heartbeat: %f", message.data);
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dog_Heart>("fdog_heart");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}