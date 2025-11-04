#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

class ImageTriggerNode : public rclcpp::Node
{
public:
  ImageTriggerNode()
  : Node("image_trigger_node"),
    threshold_(150),
    softL_(false),
    hardL_(false),
    softR_(false),
    hardR_(false),
    backward_(false)
    //triggered_(false)
  {
    // subscribe to camera image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&ImageTriggerNode::image_callback, this, std::placeholders::_1));

    // publisher for velocity
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // timer to publish velocity when triggered
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(60),
      std::bind(&ImageTriggerNode::publish_cmd, this));

    RCLCPP_INFO(this->get_logger(), "Image trigger node started. Monitoring pixel intensity...");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // center pixel location on monitor
      int rowC = 450;
      int colC = 960;
      cv::Vec3b pixelC = frame.at<cv::Vec3b>(rowC, colC);
      int intensityC = (pixelC[0] + pixelC[1] + pixelC[2]);
      bool cp = false;

      // right pixel location on monitor
      int rowR = 460;
      int colR = 1910;
      cv::Vec3b pixelR = frame.at<cv::Vec3b>(rowR, colR);
      int intensityR = (pixelR[0] + pixelR[1] + pixelR[2]);
      bool rp = false;

      // left pixel location on monitor
      int rowL = 460;
      int colL = 10;
      cv::Vec3b pixelL = frame.at<cv::Vec3b>(rowL, colL);
      int intensityL = (pixelL[0] + pixelL[1] + pixelL[2]);
      bool lp = false;


      // set booleans for wheter there is an obstacle
      if (intensityC < threshold_) {
        cp = true;
      }

      if (intensityL < threshold_) {
        lp = true;
      }

      if (intensityR < threshold_) {
        rp = true;
      }


      int code = (lp << 2) | (rp << 1) | cp;

      switch (code) {
          case 0: // lp=false, rp=false, cp=false
              RCLCPP_INFO(this->get_logger(),
                      "Forward"
                      );
              break;
          case 1: // lp=false, rp=false, cp=true
              backward_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Backward"
                      );
              break;
          case 2: // lp=false, rp=true, cp=false
              softL_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Soft Left"
                      );
              break;
          case 3: // lp=false, rp=true, cp=true
              hardL_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Hard Left"
                      );
              break;
          case 4: // lp=true, rp=false, cp=false
              softR_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Soft Right"
                      );
              break;
          case 5: // lp=true, rp=false, cp=true
              hardR_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Hard Right"
                      );
              break;
          case 6: // lp=true, rp=true, cp=false
              //backward_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Forward"
                      );
              break;
          case 7: // lp=true, rp=true, cp=true
              backward_ = true;
              RCLCPP_INFO(this->get_logger(),
                      "Backward"
                      );
              break;
          default:
              // optional 9th case, e.g. invalid data
              break;
      }



    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void publish_cmd()
  {
    if (softL_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.4;
      cmd.angular.z = 2.0;
      cmd_pub_->publish(cmd);
    }
    else if (hardL_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -0.6;
      cmd.angular.z = -2.0;
      cmd_pub_->publish(cmd);
    }
    else if (softR_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.4;
      cmd.angular.z = -2.0;
      cmd_pub_->publish(cmd);
    }
    else if (hardR_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -0.6;
      cmd.angular.z = 2.0;
      cmd_pub_->publish(cmd);
    }
    else if (backward_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -0.8;
      cmd.angular.z = 0.02;
      cmd_pub_->publish(cmd);
    }
    else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.8;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
    }

    // reset all booleans
    backward_ = false;
    softL_ = false;
    hardL_ = false;
    softR_ = false;
    hardR_ = false;

  }

  // ROS members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  int threshold_;
  bool softL_;
  bool hardL_;
  bool softR_;
  bool hardR_;
  bool backward_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
