#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Splitter : public rclcpp::Node
{
private:
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr input_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_y_pub_;

    void on_input(const asdfr_interfaces::msg::Point2 msg) {

        std_msgs::msg::Float32 x;
        std_msgs::msg::Float32 y;

        x.data = msg.x;
        y.data = msg.y;

        output_x_pub_->publish(x);
        output_y_pub_->publish(y);

        
    }

public:
    Splitter() : Node("Splitter")
    {
        input_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
        "input", 1 , std::bind(&Splitter::on_input, this, _1));

        output_x_pub_ = this->create_publisher<std_msgs::msg::Float32>("output_x", 1);
        output_y_pub_ = this->create_publisher<std_msgs::msg::Float32>("output_y", 1);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Splitter>());
    rclcpp::shutdown();
    return 0;
}