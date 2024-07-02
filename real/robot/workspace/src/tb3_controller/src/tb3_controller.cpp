#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

typedef enum {
    Tb3ControllerState_WAIT = 0,
    Tb3ControllerState_MOVE,
    Tb3ControllerState_DONE,
    Tb3ControllerState_NUM
} Tb3ControllerStateType;
typedef enum {
    SignalState_RED = 0,
    SignalState_Yellow = 1,
    SignalState_Blue = 2,
    Tb3ControllerState_NUM
} SignalStateType;

class Tb3ControllerNode : public rclcpp::Node
{
public:
    Tb3ControllerNode()
        : Node("tb3_controller_node"), state_(Tb3ControllerState_WAIT), baggage_touch_(false)
    {
        RCLCPP_INFO(this->get_logger(), "START: tb3_controller_node");

        // Publisherの作成
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/TB3RoboModel_cmd_vel", 10);

        // Subscriberの作成
        subscription_touch_ = this->create_subscription<std_msgs::msg::Bool>(
            "TB3RoboAvatar_baggage_sensor", 10,
            std::bind(&Tb3ControllerNode::sensor_callback_touch, this, std::placeholders::_1));

        subscription_signal_ = this->create_subscription<std_msgs::msg::UInt32>(
            "TB3RoboAvatar_cmd_pos", 10,
            std::bind(&Tb3ControllerNode::sensor_callback_pos, this, std::placeholders::_1));

        ///VirtualSignal_signal_data

        // タイマーの設定（例として500msごとにprocess_positionを呼び出す）
        //timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(500),
        //    std::bind(&Tb3ControllerNode::timer_callback, this));
    }

private:
    void sensor_callback_touch(const std_msgs::msg::Bool::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data ? "true" : "false");
        baggage_touch_ = msg->data;
    }
    void sensor_callback_touch(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        switch (msg->data) {
            0:
                signal_state_ = SignalState_RED;
                break;
            1:
                signal_state_ = SignalState_Yellow;
                break;
            default:
                signal_state_ = Signal_state_Blue;
                break;
        }
        RCLCPP_INFO(this->get_logger(), "Received Signal message: %d", signal_state_);
    }

    void sensor_callback_pos(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        process_position(msg->linear.x);
    }

    void timer_callback()
    {
        // テスト用の定期呼び出し
        float test_position = 0.0;  // テスト用の位置
        if (signal_state_ == SignalState_Blue) {
            process_position(test_position);
        }
        else {
            // 停止
            twist_message.linear.x = 0.0; // 停止
            twist_message.angular.z = 0.0; // 直進
        }
    }

    void process_position(float x)
    {
        auto twist_message = geometry_msgs::msg::Twist();
        const double TARGET_POS = 2.0;

        if (state_ == Tb3ControllerState_WAIT && baggage_touch_) {
            state_ = Tb3ControllerState_MOVE;
        }

        if (state_ == Tb3ControllerState_MOVE) {
            if (!baggage_touch_ || (x >= TARGET_POS)) {
                // 停止
                twist_message.linear.x = 0.0; // 停止
                twist_message.angular.z = 0.0; // 直進
                RCLCPP_INFO(this->get_logger(), "Stopping");
                if (baggage_touch_) {
                    state_ = Tb3ControllerState_DONE;
                }
            } else {
                // 移動: 前進するための速度設定
                twist_message.linear.x = 0.2; // 前進速度
                twist_message.angular.z = 0.0; // 直進
                RCLCPP_INFO(this->get_logger(), "Moving forward");
            }
            // 速度指令を発行
            publisher_->publish(twist_message);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_touch_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_pos_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_signal_;
    rclcpp::TimerBase::SharedPtr timer_;
    Tb3ControllerStateType state_;
    bool baggage_touch_;
    SignalStateType signal_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Tb3ControllerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
