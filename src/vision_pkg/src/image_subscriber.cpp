#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"


class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer() : Node("image_viewer")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&ImageViewer::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // Convert ROS image message to OpenCV image
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // add squares to the image
      int block_height = 20;
      int block_width = 20;

      // add a blue rectangle to center of the frame
      int row_start = 440;
      int col_start = 950;
      cv::Rect roi(col_start, row_start, block_width, block_height);
      frame(roi).setTo(cv::Scalar(0, 0, 255));
      // add a green rectangle to the right of the frame
      row_start = 450;
      col_start = 1900;
      cv::Rect roi1(col_start, row_start, block_width, block_height);
      frame(roi1).setTo(cv::Scalar(0, 255, 0));
      // add a red rectangle to the left of the frame
      row_start = 450;
      col_start = 0;
      cv::Rect roi2(col_start, row_start, block_width, block_height);
      frame(roi2).setTo(cv::Scalar(255, 0, 0));


      // display the image
      cv::namedWindow("Camera Feed", cv::WINDOW_NORMAL);  // allow resizing
      cv::resizeWindow("Camera Feed", 1620, 780);          // set window size
      cv::imshow("Camera Feed", frame);
      cv::waitKey(1); // refresh OpenCV window

      // print the value of one pixel every 30 frames
      if (frame_count_ % 30 == 0)
      {
        int row = 100;
        int col = 150;
        if (row < frame.rows && col < frame.cols)
        {
          cv::Vec3b pixel = frame.at<cv::Vec3b>(row, col);
          RCLCPP_INFO(this->get_logger(),
                      "Frame %d - Pixel (100,150): B=%d, G=%d, R=%d",
                      frame_count_, pixel[0], pixel[1], pixel[2]);
//           Pixel (100,150) == (column, row) == 1920x1080
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
                      "Chosen pixel (%d,%d) is outside image bounds (%d x %d)",
                      row, col, frame.rows, frame.cols);
        }
      }

      frame_count_++;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t frame_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewer>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
