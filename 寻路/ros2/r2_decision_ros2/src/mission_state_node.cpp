#include <chrono>
#include <sstream>
#include <string>

#include "r2_decision_cpp/flag_bus.hpp"
#include "r2_decision_cpp/models.hpp"
#include "r2_decision_cpp/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MissionStateNode : public rclcpp::Node {
public:
    MissionStateNode()
        : Node("mission_state_node"), sm_(bus_) {
        this->declare_parameter("tick_hz", 5.0);

        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/r2/flags/in", 100,
            std::bind(&MissionStateNode::on_flag_message, this, std::placeholders::_1));

        phase_pub_ = this->create_publisher<std_msgs::msg::String>("/r2/phase", 20);
        log_pub_ = this->create_publisher<std_msgs::msg::String>("/r2/log", 100);

        const double hz = this->get_parameter("tick_hz").as_double();
        const auto dt = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
        timer_ = this->create_wall_timer(dt, std::bind(&MissionStateNode::on_tick, this));

        bus_.publish("simulate_grab_ok", true);
        bus_.publish("simulate_qr_ok", true);
        bus_.publish("simulate_assemble_ok", true);
        bus_.publish("mf_kfs_in_box", 1);
        bus_.publish("mf_kfs_out_box_on_platform", 1);
        bus_.publish("mf_kfs_out_platform", 0);

        RCLCPP_INFO(this->get_logger(), "[任务状态机] 节点启动，开始周期tick");
    }

private:
    static bool to_bool(const std::string& value) {
        return value == "1" || value == "true" || value == "True";
    }

    void on_flag_message(const std_msgs::msg::String::SharedPtr msg) {
        const auto p = msg->data.find('=');
        if (p == std::string::npos) {
            return;
        }
        const std::string key = msg->data.substr(0, p);
        const std::string val = msg->data.substr(p + 1);

        if (key == "recollect_mf_kfs") {
            ctx_.missed_mf_kfs_on_platform = to_bool(val);
            return;
        }
        if (key == "guard_touch_fake_kfs" || key == "guard_touch_r1_kfs" ||
            key == "guard_non_adjacent_pick" || key == "guard_step_on_kfs_cell") {
            bus_.publish(key, to_bool(val));
            return;
        }

        bus_.publish(key, val);
    }

    void publish_text(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, const std::string& text) {
        std_msgs::msg::String msg;
        msg.data = text;
        pub->publish(msg);
    }

    void on_tick() {
        const auto prev_logs = ctx_.logs.size();
        const auto p = sm_.tick(ctx_);

        if (p == r2::Phase::武馆) {
            ctx_.r1_left_mc = true;
        }

        publish_text(phase_pub_, std::string("当前阶段=") + r2::to_chinese(p));

        for (size_t i = prev_logs; i < ctx_.logs.size(); ++i) {
            publish_text(log_pub_, ctx_.logs[i]);
            RCLCPP_INFO(this->get_logger(), "%s", ctx_.logs[i].c_str());
        }

        if (p == r2::Phase::结束 && !finish_reported_) {
            finish_reported_ = true;
            RCLCPP_INFO(this->get_logger(), "[任务状态机] 流程结束，携带KFS=%d", ctx_.holding_r2_kfs);
            timer_->cancel();
        }
    }

    r2::FlagBus bus_;
    r2::RobotContext ctx_;
    r2::R2DecisionStateMachine sm_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool finish_reported_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionStateNode>());
    rclcpp::shutdown();
    return 0;
}
