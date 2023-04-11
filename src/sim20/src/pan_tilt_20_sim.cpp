#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "asdfr_interfaces/msg/point2.hpp"

#include "ControllerPanTilt.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

using asdfr_interfaces::msg::Point2;

// create a node class
class Controller20sim : public rclcpp::Node
{
public:
  Controller20sim() : Node("controller_20_sim")
  {
    _cpt.Initialize(_u, _y, 0.0);
    _cpt.SetFinishTime(0.0);

    _timer = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&Controller20sim::tick, this)
    );

    _publish_timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Controller20sim::publish, this)
    );

    _position_sub = this->create_subscription<Point2>(
      "position", 
      1, 
      std::bind(&Controller20sim::on_position, this, _1)
    );

    _setpoint_sub = this->create_subscription<Point2>(
      "setpoint", 
      1, std::bind(&Controller20sim::on_setpoint, this, _1)
    );

    _motor_power_pub = this->create_publisher<Point2>(
      "motor_power", 
      1
    );
  }


private:
  ControllerPanTilt _cpt;

  // controller inputs and outputs
  XXDouble _u[4];
  XXDouble _y[2];

  // timer
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::TimerBase::SharedPtr _publish_timer;

  // subscriptions
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr _position_sub;
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr _setpoint_sub;

  // publications
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr _motor_power_pub;


  void tick() {
    _cpt.Calculate(_u, _y);
  }

  void publish() {
    auto msg = asdfr_interfaces::msg::Point2();
    msg.x = _y[0];
    msg.y = _y[1];
    _motor_power_pub->publish(msg);

  }

  void on_position(const asdfr_interfaces::msg::Point2::SharedPtr msg) {
    _u[0] = msg->x;
    _u[1] = msg->y;
  }

  void on_setpoint(const asdfr_interfaces::msg::Point2::SharedPtr msg) {
    _u[2] = msg->x;
    _u[3] = msg->y;
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller20sim>());
  rclcpp::shutdown();

  return 0;
}
