#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;
using namespace std::chrono_literals;

// Global parameters (we need them in different places). You might want to change these.
const double x_limit_rad = 0.8;
const double y_limit_rad = 0.6;
const int moving_camera_output_width = 160;
const int moving_camera_output_height = 120;

const std::chrono::milliseconds sampleTime = 50ms;


void ConvertToRad(double &x_out_rad, double &y_out_rad, const double x_in_pixels, const double y_in_pixels) {
  /* Adjust this to your own needs.
  You may want to adjust the global parameters x_limit_rad, y_limit_rad and moving_camera_output_width and moving_camera_output_height
  
  The default assumes:
  - The input position of the COG is in pixels; top left is (0,0).
  - The Jiwy Simulator settings are the default ones:
    +  Full range of motion in x direction (from far left to far right of the original
       camera image) is equivalent to a range of [-0.8.. 0.8 ] rad.
    +  Full range of motion in y direction (from far left to far right of the original
       camera image) is equivalent to a range of [-0.6.. 0.6 ] rad.
  */     

  // Explanation of the computations:
  // The default image width/height coming from jiwy_simulator (i.e., /moving_camera_output) 
  // is half of the original image width/height. Therefore, the range of motion mentioned above
  // is mapped on double the width/height of the /moving_camera_output.

  double x_rad_per_pixel = (2*x_limit_rad) / (2*moving_camera_output_width);
  double y_rad_per_pixel = (2*y_limit_rad) / (2*moving_camera_output_height);

  x_out_rad =   (x_in_pixels - moving_camera_output_width /2) * x_rad_per_pixel;
  y_out_rad = - (y_in_pixels - moving_camera_output_height/2) * y_rad_per_pixel;

}

/** Convert a chrono::milliseconds duration to a normal double variable, stating the duration in seconds.
 */
double ToSeconds(std::chrono::milliseconds t) {
   return t.count()/1000.0;
}


class ClosedLoopController : public rclcpp::Node
{
  private:
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr cog_topic_;
    rclcpp::Publisher   <asdfr_interfaces::msg::Point2>::SharedPtr setpoint_topic_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    double cog_x, cog_y;
    double setpoint_x, setpoint_y;

    void cog_input(const asdfr_interfaces::msg::Point2::SharedPtr msg)
    {
      // Fill cog_x and cog_y with converted values (from pixels to radians)
      ConvertToRad(cog_x, cog_y, msg->x, msg->y);
      RCLCPP_INFO(this->get_logger(), "COG position: (%f, %f) pixels = (%f, %f) rad", msg->x, msg->y, cog_x, cog_y);
    }

    void control_loop_timer_callback()
    {
      // Do the control loop 
      double tau = this->get_parameter("tau_s").as_double();

      // Refer to (1.2) in the Student Manual. Notice that cog_x/y is exactly  (x_light-x_jiwy).
      // Thus we only need to multiply with 1/tau.
      double ddt_setpoint_x = 1/tau * cog_x; // Derivative
      double ddt_setpoint_y = 1/tau * cog_y; // Derivative

      // Forward euler integration
      setpoint_x += ddt_setpoint_x * ToSeconds(sampleTime);
      setpoint_y += ddt_setpoint_y * ToSeconds(sampleTime);

      // Limit the setpoint
      if (setpoint_x < -x_limit_rad) setpoint_x = -x_limit_rad;
      if (setpoint_x >  x_limit_rad) setpoint_x =  x_limit_rad;
      if (setpoint_y < -y_limit_rad) setpoint_y = -y_limit_rad;
      if (setpoint_y >  y_limit_rad) setpoint_y =  y_limit_rad;

      // Output the actual position
      asdfr_interfaces::msg::Point2 setpoint;
      setpoint.x = setpoint_x;
      setpoint.y = setpoint_y;
      setpoint_topic_->publish(setpoint);
    }

  public:
    ClosedLoopController()
    : Node("ClosedLoopController"),
      cog_x(0), cog_y(0),
      setpoint_x(0), setpoint_y(0)
    {
      // Create listener to COG channel
      cog_topic_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
        "cog_input", 1 , std::bind(&ClosedLoopController::cog_input, this, _1));

      // Create publisher for Setpoint channel.
      setpoint_topic_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint_output", 1);
      
      // We always want to output something, even if we don't get data from the COG.
      // Therefore, it is a timer callback.
      control_loop_timer_ = this->create_wall_timer( sampleTime, 
              std::bind(&ClosedLoopController::control_loop_timer_callback, this)); 
  
      // parameter tau (in seconds)
      this->declare_parameter<double>("tau_s", 1.0);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}