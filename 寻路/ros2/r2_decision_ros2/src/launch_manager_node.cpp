#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LaunchManagerNode : public rclcpp::Node {
public:
    LaunchManagerNode() : Node("launch_manager_node") {
        this->declare_parameter("cold_start", true);
        this->declare_parameter("rebuild_weapon", false);
        this->declare_parameter("recollect_mf_kfs", false);

        pub_ = this->create_publisher<std_msgs::msg::String>("/r2/flags/in", 50);
        timer_ = this->create_wall_timer(1000ms, std::bind(&LaunchManagerNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "[启动管理] 节点启动，准备发布启动分流flag");
    }

private:
    void publish_flag(const std::string& key, const std::string& value) {
        std_msgs::msg::String msg;
        msg.data = key + "=" + value;
        pub_->publish(msg);
    }

    void on_timer() {
        const bool cold_start = this->get_parameter("cold_start").as_bool();
        const bool rebuild = this->get_parameter("rebuild_weapon").as_bool();
        const bool recollect = this->get_parameter("recollect_mf_kfs").as_bool();

        publish_flag("cold_start", cold_start ? "1" : "0");
        publish_flag("rebuild_weapon", rebuild ? "1" : "0");
        publish_flag("recollect_mf_kfs", recollect ? "1" : "0");

        RCLCPP_INFO(this->get_logger(), "[启动管理] 已发布模式flag：cold=%d rebuild=%d recollect=%d",
                    static_cast<int>(cold_start), static_cast<int>(rebuild), static_cast<int>(recollect));
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaunchManagerNode>());
    rclcpp::shutdown();
    return 0;
}
