#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/imu.hpp"

typedef enum {
    Tb3ControllerState_WAIT = 0,
    Tb3ControllerState_MOVE,
    Tb3ControllerState_DONE,
    Tb3ControllerState_BACK,
    Tb3ControllerState_NUM
} Tb3ControllerStateType;
typedef enum {
    SignalState_RED = 0,
    SignalState_Yellow = 1,
    SignalState_Blue = 2,
    SignalState_NUM
} SignalStateType;
#define MOTOR_POWER 0.2
#define ANGULAR_MIN -0.1
#define ANGULAR_MAX 0.1
#include <cmath>

class PID
{
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double calculate(double setpoint, double pv)
    {
        double error = setpoint - pv;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

class Tb3ControllerNode : public rclcpp::Node
{
public:
    Tb3ControllerNode()
        : Node("tb3_controller_node"), state_(Tb3ControllerState_WAIT), baggage_touch_(false),
         pid_controller_(1.0, 0.0, 0.1)
    {
        // パラメータを宣言し、デフォルト値を設定
        this->declare_parameter<std::string>("act_mode", "sim");
        std::string act_mode = this->get_parameter("act_mode").as_string();

        // act_modeに基づいてトピック名を切り替える
        std::string ros_topic_name_cmd_vel = "/tb3_cmd_vel";
        std::string ros_topic_name_imu = "/tb3_imu";
        if (act_mode == "sim") {
            ros_topic_name_cmd_vel = "/TB3RoboModel_cmd_vel";
            ros_topic_name_imu = "/TB3RoboModel_imu";
        }
        
        RCLCPP_INFO(this->get_logger(), "START: tb3_controller_node: %s", act_mode.c_str());

        // Publisherの作成
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(ros_topic_name_cmd_vel, 10);

        // Subscriberの作成
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            ros_topic_name_imu, 10,
            std::bind(&Tb3ControllerNode::sensor_callback_imu, this, std::placeholders::_1));
        subscription_touch_ = this->create_subscription<std_msgs::msg::Bool>(
            "TB3RoboAvatar_baggage_sensor", 10,
            std::bind(&Tb3ControllerNode::sensor_callback_touch, this, std::placeholders::_1));

        ///VirtualSignal_signal_data
        subscription_signal_ = this->create_subscription<std_msgs::msg::UInt32>(
            "VirtualSignal_signal_data", 10,
            std::bind(&Tb3ControllerNode::sensor_callback_signal, this, std::placeholders::_1));


        subscription_pos_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "TB3RoboAvatar_cmd_pos", 10,
            std::bind(&Tb3ControllerNode::sensor_callback_pos, this, std::placeholders::_1));
    }

private:

    std::array<double, 3> euler_from_quaternion(double x, double y, double z, double w) {
        std::array<double, 3> euler;

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            euler[1] = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    void sensor_callback_touch(const std_msgs::msg::Bool::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data ? "true" : "false");
        baggage_touch_ = msg->data;
    }
    void sensor_callback_signal(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        switch (msg->data) {
            case 0:
                signal_state_ = SignalState_RED;
                break;
            case 1:
                signal_state_ = SignalState_Yellow;
                break;
            default:
                signal_state_ = SignalState_Blue;
                break;
        }
        //RCLCPP_INFO(this->get_logger(), "Received Signal message: %d", signal_state_);
    }
    void stop_motor()
    {
        auto twist_message = geometry_msgs::msg::Twist();
        // 停止
        twist_message.linear.x = 0.0; // 停止
        twist_message.angular.z = 0.0; // 直進
        publisher_->publish(twist_message);
    }

    void sensor_callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        // クォータニオンからオイラー角への変換
        std::array<double, 3> euler = euler_from_quaternion(qx, qy, qz, qw);

        // current_yaw_を更新
        current_yaw_ = euler[2];
    }
    void sensor_callback_pos(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float pos = msg->linear.x;
        bool need_signal_control = false;

        if (state_ == Tb3ControllerState_MOVE) {
            if (pos >= 0.5 && pos <= 0.8) { //before cross road
                need_signal_control = true;
            }
        }
        else if (state_ == Tb3ControllerState_BACK) {
            if (pos >= 1.3 && pos <= 1.8) { //before cross road
                need_signal_control = true;
            }
        }

        if (need_signal_control) {
            if (signal_state_ == SignalState_Blue) {
                //RCLCPP_INFO(this->get_logger(), "BLUE");
                process_position(pos);
            }
            else {
                //RCLCPP_INFO(this->get_logger(), "RED: %d", signal_state_);
                stop_motor();
            }
        }
        else {
            process_position(pos);
        }
    }

    void process_rotation(float target_yaw, float current_yaw, geometry_msgs::msg::Twist& twist_message)
    {
        // PID制御で角速度を計算
        double angular_z = pid_controller_.calculate(target_yaw, current_yaw);
        if (angular_z > ANGULAR_MAX) {
            angular_z = ANGULAR_MAX;
        }
        if (angular_z < ANGULAR_MIN) {
            angular_z = ANGULAR_MIN;
        }

        // twist_messageに角速度を設定
        twist_message.angular.z = angular_z;

        // デバッグ用出力
        //std::cout << "Target Yaw: " << target_yaw << ", Current Yaw: " << current_yaw << ", Angular Z: " << angular_z << std::endl;
    }
    void process_position(float x)
    {
        auto twist_message = geometry_msgs::msg::Twist();
        const double TARGET_POS = 2.0;

        if (state_ == Tb3ControllerState_WAIT && baggage_touch_) {
            state_ = Tb3ControllerState_MOVE;
        }
        else if (state_ == Tb3ControllerState_DONE && !baggage_touch_) {
            state_ = Tb3ControllerState_BACK;
        }

        if (state_ == Tb3ControllerState_BACK) {
            if (x <= 0.5) {
                // 停止
                twist_message.linear.x = 0.0; // 停止
                RCLCPP_INFO(this->get_logger(), "Waiting mode");
                state_ = Tb3ControllerState_WAIT;
            } else {
                // 移動: 前進するための速度設定
                twist_message.linear.x =  -MOTOR_POWER;
                RCLCPP_INFO(this->get_logger(), "Moving backward");
            }
            process_rotation(0, current_yaw_, twist_message);//TODO
            // 速度指令を発行
            publisher_->publish(twist_message);
        }
        else if (state_ == Tb3ControllerState_MOVE) {
            if (!baggage_touch_ || (x >= TARGET_POS)) {
                // 停止
                twist_message.linear.x = 0.0; // 停止
                RCLCPP_INFO(this->get_logger(), "Stopping");
                if (baggage_touch_) {
                    state_ = Tb3ControllerState_DONE;
                }
            } else {
                // 移動: 前進するための速度設定
                twist_message.linear.x =  MOTOR_POWER; // 前進速度
                RCLCPP_INFO(this->get_logger(), "Moving forward");
            }
            process_rotation(0, current_yaw_, twist_message);//TODO
            // 速度指令を発行
            publisher_->publish(twist_message);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_touch_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_pos_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_signal_;
    rclcpp::TimerBase::SharedPtr timer_;
    Tb3ControllerStateType state_;
    bool baggage_touch_;
    float current_yaw_;
    SignalStateType signal_state_;
    PID pid_controller_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Tb3ControllerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
