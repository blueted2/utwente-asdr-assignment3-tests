#include <cstdio>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

#include "image_functions.h"

using std::placeholders::_1;

class LightPos : public rclcpp::Node
{
  public:
    LightPos() : 
      Node("brightness_threshold") 
    {
      // Listen to image topic /image
      RCLCPP_INFO(this->get_logger(), "listening for images at /image. Try:");
      RCLCPP_INFO(this->get_logger(), "  ros2 run image_tools cam2image");      
      subscriptionImage_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&LightPos::topic_callback_image, this, _1));

      // Create a thresholded topic
      RCLCPP_INFO(this->get_logger(), "Creating /image_thresholded topic. Try:");
      RCLCPP_INFO(this->get_logger(), "  ros2 run image_tools show /image:=/image_thresholded");
      image_thresholded_topic_ = this->create_publisher<sensor_msgs::msg::Image>("image_thresholded",1);

      // Cog-pos
      RCLCPP_INFO(this->get_logger(), "Creating /cog_pos. Try:");
      RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /cog_pos");
      cog_pos_topic_ = this->create_publisher<asdfr_interfaces::msg::Point2>("cog_pos",1);


      // Create a brightness_threshold parameter (initialized to 240)
      RCLCPP_INFO(this->get_logger(), "Creating brightness_threshold parameter. Try:");
      RCLCPP_INFO(this->get_logger(), "  ros2 param set /light_pos brightness_threshold 230");
      this->declare_parameter<int>("brightness_threshold", 240);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionImage_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr cog_pos_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_topic_;

    void topic_callback_image(const sensor_msgs::msg::Image::SharedPtr img) const
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Received first image!");
      RCLCPP_INFO_ONCE(this->get_logger(), "   Encoding = %s", img->encoding.c_str());
      RCLCPP_INFO_ONCE(this->get_logger(), "   W x H = %d x %d", img->width, img->height);
      // RCLCPP_INFO(this->get_logger(), "step = %d", img->step);
      // RCLCPP_INFO(this->get_logger(), "bigEndian = %d", img->is_bigendian);
      // RCLCPP_INFO(this->get_logger(), "frame-id = %s", img->header.frame_id.c_str());

      int brightness_threshold=0; // No real need to intialize because it is set below, but this saves an "uninitialized" warning
      this->get_parameter("brightness_threshold", brightness_threshold);

      // Create a thresholded image; both as an intermediate step as a 'debug' output.
      // On the same run, compute the center of gravity.
      auto thr_image = sensor_msgs::msg::Image::SharedPtr(new sensor_msgs::msg::Image());
      copyImageProperties(thr_image, img);
      int brightness, thresholdColor=255;
      // Computation of COG = 1/N * sum(x_n) (where n=1..N; x_n is the x position of the n'th pixel that is above the threshold)
      double cog_sum_x = 0 , cog_sum_y = 0; // In these variables we store  sum(x_n) and sum(y_n)
      int cog_N = 0;


      for (int y=0; y<getImageHeight(img); y++) { // Loop over rows
        for (int x=0; x<getImageWidth(img); x++) { // Loop over columns
          brightness = getPixelBrightness(img, x, y);    
          if (brightness >= brightness_threshold)
          {
            setPixelColor(thr_image, x, y, thresholdColor, thresholdColor, thresholdColor);
            cog_sum_x += x;
            cog_sum_y += y;
            cog_N++;
          } else 
            setPixelColor(thr_image, x, y, 0, 0, 0);
        }
      }

      // Identify COG (if any). Put it in the image as a 3x3 square of dots
      // (unless it is too close to the edge)
      int cog_x=0, cog_y=0;
      if (cog_N!=0)
      {
        cog_x = int(round(cog_sum_x / cog_N));
        cog_y = int(round(cog_sum_y / cog_N));
        DrawBigPixel(thr_image, cog_x, cog_y, 255,0,0,5);
      }

      asdfr_interfaces::msg::Point2 cog_msg;
      cog_msg.x = cog_x;
      cog_msg.y = cog_y;
      cog_pos_topic_->publish(cog_msg);

      // For debugging purposes
      image_thresholded_topic_->publish(*thr_image);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPos>());
  rclcpp::shutdown();
  return 0;
}
