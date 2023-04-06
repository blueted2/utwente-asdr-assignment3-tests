#include <stdexcept>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "image_functions.h"

#define MIN_INT(a,b) ((((int)(a))<((int)(b)))?((int)(a)):((int)(b)))
#define MAX_INT(a,b) ((((int)(a))>((int)(b)))?((int)(a)):((int)(b)))

/**
 * @brief Throw an out_of_range error if the the coordinate (x,y) is not within the image size
 * 
 * Throw an out_of_range error if the the coordinate (x,y) is not within the image size.
 * If the coordinate is within the image size, the function simply returns; allowing 
 * continuation of the execution.
 *
 * @param im The image we're looking at
 * @param x X-coordinate
 * @param y Y-coordinate
 * @return nothing
 */
void assertCoordinateWithinRange(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y)
{
    // Cast unsigned stuff to signed to avoid compiler warnings
    if (x < 0 || x >= (int)im->width) {
        throw std::out_of_range("x-coordinate out of range");
    }
        if (y < 0 || y >= (int)im->height) {
    throw std::out_of_range("y-coordinate out of range");
}
}

/**
 * @brief Throw a runtime error if the image type is not supported
 * 
 * This function checks a few aspects of the image (e.g., number of color 
 * channels). If they are outside of the comfortable range of all the 
 * functions, a runtime error is thrown. If the image type is supported,
 * the function simply returns; allowing continuation of the execution.
 * 
 * @param im The image in question.
 * @return nothing
 */
void assertSupportedImageType(sensor_msgs::msg::Image::ConstSharedPtr im)
{
  if (sensor_msgs::image_encodings::numChannels(im->encoding) != 3) {
    throw std::runtime_error("Unsupported image type (number of color channels is not equal to 3)");
  }
  if (sensor_msgs::image_encodings::bitDepth(im->encoding) != 8) {
    throw std::runtime_error("Unsupported image type (bit depth (number of bits per channel per pixel is not equal to 8)");
  }
}


int getPixelBrightness(sensor_msgs::msg::Image::ConstSharedPtr im, int x, int y)
{
  assertSupportedImageType(im);
  assertCoordinateWithinRange(im, x, y);

  // Each line of the image takes im->step bytes. In order to access the y'th row, we should skip y*im->step bytes.
  u_int8_t ch1_value = im->data[ im->step * y + 3*x + 0];
  u_int8_t ch2_value = im->data[ im->step * y + 3*x + 1];
  u_int8_t ch3_value = im->data[ im->step * y + 3*x + 2];

  return (int) round( (ch1_value + ch2_value + ch3_value) / 3.0);
}


void setPixelColor(sensor_msgs::msg::Image::SharedPtr im, int x, int y, int ch1_value, int ch2_value, int ch3_value)
{
  assertSupportedImageType(im);
  try {
    assertCoordinateWithinRange(im, x, y);
    // Each line of the image takes im->step bytes. In order to access the y'th row, we should skip y*im->step bytes.
    im->data[ im->step * y + 3*x + 0]= ch1_value;
    im->data[ im->step * y + 3*x + 1]= ch2_value;
    im->data[ im->step * y + 3*x + 2]= ch3_value;
  }
  catch (std::runtime_error &e) {
    std::cout<<"Warning: tried to draw a pixel at ("<<x<<","<<y<<") but that's outside the image canvas"<<std::endl;
  }

}

void DrawBigPixel(sensor_msgs::msg::Image::SharedPtr img, int x, int y, int ch1_vaule, int ch2_value, int ch3_vaule, int size) {
  // Works best if size is odd.
  int halfSize = (size-1)/2; // no comment :-)

  for (int xx=MAX_INT(0, x-halfSize); xx<=MIN_INT(img->width-1, x+halfSize); xx++)
  {
    for (int yy=MAX_INT(0, y-halfSize); yy<=MIN_INT(img->height-1, y+halfSize); yy++)
    {
      setPixelColor(img,xx,yy, ch1_vaule,ch2_value,ch3_vaule);
    }
  }
}

void copyImageProperties (sensor_msgs::msg::Image::SharedPtr dst, sensor_msgs::msg::Image::ConstSharedPtr src)
{
  *dst = *src;
}


int getImageWidth(sensor_msgs::msg::Image::ConstSharedPtr im)
{
  return (int)im->width;
}

int getImageHeight(sensor_msgs::msg::Image::ConstSharedPtr im)
{
  return (int)im->height;
}

