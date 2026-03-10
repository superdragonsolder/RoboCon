#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SafetyGuardNode : public rclcpp::Node {
public:
    SafetyGuardNode() : Node("safety_guard_node") {
        this->declare_parameter("guard_touch_fake_kfs", false);
        this->declare_parameter("guard_touch_r1_kfs", false);
        this->declare_parameter("guard_non_adjacent_pick", false);
        this->declare_parameter("guard_step_on_kfs_cell", false);

        pub_ = this->create_publisher<std_msgs::msg::String>("/r2/flags/in", 50);
        timer_ = this->create_wall_timer(400ms, std::bind(&SafetyGuardNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "[安全守卫] 节点启动，守卫flag将周期发布");
    }

private:
    void publish_flag(const std::string& key, bool value) {
        std_msgs::msg::String msg;
        msg.data = key + std::string("=") + (value ? "1" : "0");
        pub_->publish(msg);
    }

    void on_timer() {
        const bool f1 = this->get_parameter("guard_touch_fake_kfs").as_bool();
        const bool f2 = this->get_parameter("guard_touch_r1_kfs").as_bool();
        const bool f3 = this->get_parameter("guard_non_adjacent_pick").as_bool();
        const bool f4 = this->get_parameter("guard_step_on_kfs_cell").as_bool();

        publish_flag("guard_touch_fake_kfs", f1);
        publish_flag("guard_touch_r1_kfs", f2);
        publish_flag("guard_non_adjacent_pick", f3);
        publish_flag("guard_step_on_kfs_cell", f4);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyGuardNode>());
    rclcpp::shutdown();
    return 0;
}
