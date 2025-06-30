#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Caminar : public rclcpp::Node {
public:
    Caminar()
    : Node("caminar"),
      k_(0.0)
    {
        // Permitir cambiar el intervalo y frecuencia como parámetros
        this->declare_parameter<double>("freq", 0.5); // Hz
        this->declare_parameter<int>("dt_ms", 100);    // milisegundos

        this->get_parameter("freq", freq_);
        int dt_ms;
        this->get_parameter("dt_ms", dt_ms);

        dt_ = std::chrono::milliseconds(dt_ms);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        timer_ = this->create_wall_timer(dt_, std::bind(&Caminar::timer_callback, this));

        joint_state_msg_.name = {
            "Br_joint_1", "Br_joint_2", "Br_joint_3",
            "Fr_joint_1", "Fr_joint_2", "Fr_joint_3",
            "Bl_joint_1", "Bl_joint_2", "Bl_joint_3",
            "Fl_joint_1", "Fl_joint_2", "Fl_joint_3"
        };
        joint_state_msg_.position.resize(12, 0.0);
    }

private:
    void timer_callback() {
        double t = k_ * (dt_.count() / 1000.0); // t en segundos
        double phase = M_PI; // 180 grados de desfase

        // Grupo 1: Br y Fl
        joint_state_msg_.position[0] = 0.7 * std::sin(2*M_PI*freq_ * t);
        joint_state_msg_.position[1] = 0.3 * std::sin(2*M_PI*freq_ * t);
        joint_state_msg_.position[2] = 0.3 * std::sin(2*M_PI*freq_ * t);

        joint_state_msg_.position[9] = 0.7 * std::sin(2*M_PI*freq_ * t);
        joint_state_msg_.position[10]= 0.3 * std::sin(2*M_PI*freq_ * t);
        joint_state_msg_.position[11]= 0.3 * std::sin(2*M_PI*freq_ * t);

        // Grupo 2: Fr y Bl (desfasados)
        joint_state_msg_.position[3] = 0.7 * std::sin(2*M_PI*freq_ * t + phase);
        joint_state_msg_.position[4] = 0.3 * std::sin(2*M_PI*freq_ * t + phase);
        joint_state_msg_.position[5] = 0.3 * std::sin(2*M_PI*freq_ * t + phase);

        joint_state_msg_.position[6] = 0.7 * std::sin(2*M_PI*freq_ * t + phase);
        joint_state_msg_.position[7] = 0.3 * std::sin(2*M_PI*freq_ * t + phase);
        joint_state_msg_.position[8] = 0.3 * std::sin(2*M_PI*freq_ * t + phase);

        joint_state_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(joint_state_msg_);
        k_ += 1.0;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
    std::chrono::milliseconds dt_;

    double freq_; // Frecuencia (Hz)
    double k_;    // Contador de iteraciones
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Caminar>());
    rclcpp::shutdown();
    return 0;
}
